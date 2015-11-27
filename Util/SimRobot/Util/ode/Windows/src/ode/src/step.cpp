/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

#include <ode/odeconfig.h>
#include <ode/rotation.h>
#include <ode/timer.h>
#include <ode/error.h>
#include "config.h"
#include "odemath.h"
#include "matrix.h"
#include "objects.h"
#include "joints/joint.h"
#include "lcp.h"
#include "util.h"
#include "threadingutils.h"

#include <new>


#define dMIN(A,B)  ((A)>(B) ? (B) : (A))
#define dMAX(A,B)  ((B)>(A) ? (B) : (A))

//****************************************************************************
// misc defines

//#define TIMING


#ifdef TIMING
#define IFTIMING(x) x
#else
#define IFTIMING(x) ((void)0)
#endif


struct dJointWithInfo1
{
    dxJoint *joint;
    dxJoint::Info1 info;
};

struct dxStepperStage0Outputs
{
    size_t                          ji_start;
    size_t                          ji_end;
    unsigned int                    m;
    unsigned int                    nub;
};

struct dxStepperStage1CallContext
{
    dxStepperStage1CallContext(const dxStepperProcessingCallContext *stepperCallContext, void *stageMemArenaState, dReal *invI, dJointWithInfo1 *jointinfos):
        m_stepperCallContext(stepperCallContext), m_stageMemArenaState(stageMemArenaState), 
        m_invI(invI), m_jointinfos(jointinfos)
    {
    }

    const dxStepperProcessingCallContext *m_stepperCallContext;
    void                            *m_stageMemArenaState;
    dReal                           *m_invI;
    dJointWithInfo1                 *m_jointinfos;
    dxStepperStage0Outputs          m_stage0Outputs;
};

struct dxStepperStage0BodiesCallContext
{
    dxStepperStage0BodiesCallContext(const dxStepperProcessingCallContext *stepperCallContext, dReal *invI):
        m_stepperCallContext(stepperCallContext), m_invI(invI), 
        m_tagsTaken(0), m_gravityTaken(0), m_inertiaBodyIndex(0)
    {
    }

    const dxStepperProcessingCallContext *m_stepperCallContext;
    dReal                           *m_invI;
    atomicord32                     m_tagsTaken;
    atomicord32                     m_gravityTaken;
    unsigned int                    volatile m_inertiaBodyIndex;
};

struct dxStepperStage0JointsCallContext
{
    dxStepperStage0JointsCallContext(const dxStepperProcessingCallContext *stepperCallContext, dJointWithInfo1 *jointinfos, dxStepperStage0Outputs *stage0Outputs):
        m_stepperCallContext(stepperCallContext), m_jointinfos(jointinfos), m_stage0Outputs(stage0Outputs)
    {
    }

    const dxStepperProcessingCallContext *m_stepperCallContext;
    dJointWithInfo1                 *m_jointinfos;
    dxStepperStage0Outputs          *m_stage0Outputs;
};

static int dxStepIsland_Stage0_Bodies_Callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
static int dxStepIsland_Stage0_Joints_Callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
static int dxStepIsland_Stage1_Callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);

static void dxStepIsland_Stage0_Bodies(dxStepperStage0BodiesCallContext *callContext);
static void dxStepIsland_Stage0_Joints(dxStepperStage0JointsCallContext *callContext);
static void dxStepIsland_Stage1(dxStepperStage1CallContext *callContext);


struct dxStepperLocalContext
{
    void Initialize(dReal *invI, dJointWithInfo1 *jointinfos, unsigned int nj, 
        unsigned int m, unsigned int nub, const unsigned int *mindex, int *findex, 
        dReal *lo, dReal *hi, dReal *J, dReal *A, dReal *rhs)
    {
        m_invI = invI;
        m_jointinfos = jointinfos;
        m_nj = nj;
        m_m = m;
        m_nub = nub;
        m_mindex = mindex;
        m_findex = findex; 
        m_lo = lo;
        m_hi = hi;
        m_J = J;
        m_A = A;
        m_rhs = rhs;
    }

    dReal                           *m_invI;
    dJointWithInfo1                 *m_jointinfos;
    unsigned int                    m_nj;
    unsigned int                    m_m;
    unsigned int                    m_nub;
    const unsigned int              *m_mindex;
    int                             *m_findex;
    dReal                           *m_lo;
    dReal                           *m_hi;
    dReal                           *m_J;
    dReal                           *m_A;
    dReal                           *m_rhs;
};

struct dxStepperStage3CallContext
{
    void Initialize(const dxStepperProcessingCallContext *callContext, const dxStepperLocalContext *localContext, 
        void *stage1MemArenaState)
    {
        m_stepperCallContext = callContext;
        m_localContext = localContext;
        m_stage1MemArenaState = stage1MemArenaState;
    }

    const dxStepperProcessingCallContext *m_stepperCallContext;
    const dxStepperLocalContext     *m_localContext;
    void                            *m_stage1MemArenaState;
};

struct dxStepperStage2CallContext
{
    void Initialize(const dxStepperProcessingCallContext *callContext, const dxStepperLocalContext *localContext, 
        dReal *JinvM, dReal *rhs_tmp_or_cfm)
    {
        m_stepperCallContext = callContext;
        m_localContext = localContext;
        m_JinvM = JinvM;
        m_rhs_tmp_or_cfm = rhs_tmp_or_cfm;
        m_ji_J = 0;
        m_ji_Ainit = 0;
        m_ji_JinvM = 0;
        m_ji_Aaddjb = 0;
        m_bi_rhs_tmp = 0;
        m_ji_rhs = 0;
    }

    const dxStepperProcessingCallContext *m_stepperCallContext;
    const dxStepperLocalContext     *m_localContext;
    dReal                           *m_JinvM;
    dReal                           *m_rhs_tmp_or_cfm;
    volatile unsigned               m_ji_J;
    volatile unsigned               m_ji_Ainit;
    volatile unsigned               m_ji_JinvM;
    volatile unsigned               m_ji_Aaddjb;
    volatile unsigned               m_bi_rhs_tmp;
    volatile unsigned               m_ji_rhs;
};

static int dxStepIsland_Stage2a_Callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
static int dxStepIsland_Stage2aSync_Callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
static int dxStepIsland_Stage2b_Callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
static int dxStepIsland_Stage2bSync_Callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
static int dxStepIsland_Stage2c_Callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);
static int dxStepIsland_Stage3_Callback(void *callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee);

static void dxStepIsland_Stage2a(dxStepperStage2CallContext *callContext);
static void dxStepIsland_Stage2b(dxStepperStage2CallContext *callContext);
static void dxStepIsland_Stage2c(dxStepperStage2CallContext *callContext);
static void dxStepIsland_Stage3(dxStepperStage3CallContext *callContext);


//****************************************************************************
// special matrix multipliers


// this assumes the 4th and 8th rows of B and C are zero.

static void MultiplyAdd2_p8r (dReal *A, const dReal *B, const dReal *C,
                              unsigned int p, unsigned int r, unsigned int Askip)
{
    dIASSERT (p>0 && r>0 && A && B && C);
    const unsigned int Askip_munus_r = Askip - r;
    dIASSERT(Askip >= r);
    dReal *aa = A;
    const dReal *bb = B;
    for (unsigned int i = p; i != 0; --i) {
        const dReal *cc = C;
        for (unsigned int j = r; j != 0; --j) {
            dReal sum;
            sum  = bb[0]*cc[0];
            sum += bb[1]*cc[1];
            sum += bb[2]*cc[2];
            sum += bb[4]*cc[4];
            sum += bb[5]*cc[5];
            sum += bb[6]*cc[6];
            *(aa++) += sum; 
            cc += 8;
        }
        bb += 8;
        aa += Askip_munus_r;
    }
}


// this assumes the 4th and 8th rows of B are zero.

static void MultiplySub0_p81 (dReal *A, const dReal *B, const dReal *C, unsigned int p)
{
    dIASSERT (p>0 && A && B && C);
    dReal *aa = A;
    const dReal *bb = B;
    const dReal C_0 = C[0], C_1 = C[1], C_2 = C[2], C_4 = C[4], C_5 = C[5], C_6 = C[6];
    for (unsigned int i = p; i != 0; --i) {
        dReal sum;
        sum  = bb[0]*C_0;
        sum += bb[1]*C_1;
        sum += bb[2]*C_2;
        sum += bb[4]*C_4;
        sum += bb[5]*C_5;
        sum += bb[6]*C_6;
        *(aa++) -= sum;
        bb += 8;
    }
}


// this assumes the 4th and 8th rows of B are zero.

static void MultiplyAdd1_8q1 (dReal *A, const dReal *B, const dReal *C, unsigned int q)
{
    dIASSERT (q>0 && A && B && C);
    const dReal *bb = B;
    dReal sum0 = 0, sum1 = 0, sum2 = 0, sum4=0, sum5 = 0, sum6 = 0;
    for (unsigned int k = 0; k < q; ++k) {
        const dReal C_k = C[k];
        sum0 += bb[0] * C_k;
        sum1 += bb[1] * C_k;
        sum2 += bb[2] * C_k;
        sum4 += bb[4] * C_k;
        sum5 += bb[5] * C_k;
        sum6 += bb[6] * C_k;
        bb += 8;
    }
    A[0] += sum0;
    A[1] += sum1;
    A[2] += sum2;
    A[4] += sum4;
    A[5] += sum5;
    A[6] += sum6;
}


// this assumes the 4th and 8th rows of B are zero.

static void Multiply1_8q1 (dReal *A, const dReal *B, const dReal *C, unsigned int q)
{
    const dReal *bb = B;
    dReal sum0 = 0, sum1 = 0, sum2 = 0, sum4=0, sum5 = 0, sum6 = 0;
    for (unsigned int k = 0; k < q; ++k) {
        const dReal C_k = C[k];
        sum0 += bb[0] * C_k;
        sum1 += bb[1] * C_k;
        sum2 += bb[2] * C_k;
        sum4 += bb[4] * C_k;
        sum5 += bb[5] * C_k;
        sum6 += bb[6] * C_k;
        bb += 8;
    }
    A[0] = sum0;
    A[1] = sum1;
    A[2] = sum2;
    A[4] = sum4;
    A[5] = sum5;
    A[6] = sum6;
}

//****************************************************************************

/*extern */
void dxStepIsland(const dxStepperProcessingCallContext *callContext)
{
    IFTIMING(dTimerStart("preprocessing"));

    dxWorldProcessMemArena *memarena = callContext->m_stepperArena;
    dxWorld *world = callContext->m_world;
    unsigned int nb = callContext->m_islandBodiesCount;
    unsigned int _nj = callContext->m_islandJointsCount;

    dReal *invI = memarena->AllocateArray<dReal>(3 * 4 * (size_t)nb);
    // Reserve twice as much memory and start from the middle so that regardless of 
    // what direction the array grows to there would be sufficient room available.
    const size_t ji_reserve_count = 2 * (size_t)_nj;
    dJointWithInfo1 *const jointinfos = memarena->AllocateArray<dJointWithInfo1>(ji_reserve_count);

    const unsigned allowedThreads = callContext->m_stepperAllowedThreads;
    dIASSERT(allowedThreads != 0);

    void *stagesMemArenaState = memarena->SaveState();

    dxStepperStage1CallContext *stage1CallContext = (dxStepperStage1CallContext *)memarena->AllocateBlock(sizeof(dxStepperStage1CallContext));
    new(stage1CallContext) dxStepperStage1CallContext(callContext, stagesMemArenaState, invI, jointinfos);

    dxStepperStage0BodiesCallContext *stage0BodiesCallContext = (dxStepperStage0BodiesCallContext *)memarena->AllocateBlock(sizeof(dxStepperStage0BodiesCallContext));
    new(stage0BodiesCallContext) dxStepperStage0BodiesCallContext(callContext, invI);
    
    dxStepperStage0JointsCallContext *stage0JointsCallContext = (dxStepperStage0JointsCallContext *)memarena->AllocateBlock(sizeof(dxStepperStage0JointsCallContext));
    new(stage0JointsCallContext) dxStepperStage0JointsCallContext(callContext, jointinfos, &stage1CallContext->m_stage0Outputs);

    if (allowedThreads == 1)
    {
        dxStepIsland_Stage0_Bodies(stage0BodiesCallContext);
        dxStepIsland_Stage0_Joints(stage0JointsCallContext);
        dxStepIsland_Stage1(stage1CallContext);
    }
    else
    {
        unsigned bodyThreads = allowedThreads;
        unsigned jointThreads = 1;

        dCallReleaseeID stage1CallReleasee;
        world->PostThreadedCallForUnawareReleasee(NULL, &stage1CallReleasee, bodyThreads + jointThreads, callContext->m_finalReleasee, 
            NULL, &dxStepIsland_Stage1_Callback, stage1CallContext, 0, "StepIsland Stage1");

        world->PostThreadedCallsGroup(NULL, bodyThreads, stage1CallReleasee, &dxStepIsland_Stage0_Bodies_Callback, stage0BodiesCallContext, "StepIsland Stage0-Bodies");

        world->PostThreadedCall(NULL, NULL, 0, stage1CallReleasee, NULL, &dxStepIsland_Stage0_Joints_Callback, stage0JointsCallContext, 0, "StepIsland Stage0-Joints");
        dIASSERT(jointThreads == 1);
    }
}    

static 
int dxStepIsland_Stage0_Bodies_Callback(void *_callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee)
{
    (void)callInstanceIndex; // unused
    (void)callThisReleasee; // unused
    dxStepperStage0BodiesCallContext *callContext = (dxStepperStage0BodiesCallContext *)_callContext;
    dxStepIsland_Stage0_Bodies(callContext);
    return 1;
}

static 
void dxStepIsland_Stage0_Bodies(dxStepperStage0BodiesCallContext *callContext)
{
    dxBody * const *body = callContext->m_stepperCallContext->m_islandBodiesStart;
    unsigned int nb = callContext->m_stepperCallContext->m_islandBodiesCount;

    if (ThrsafeExchange(&callContext->m_tagsTaken, 1) == 0)
    {
        // number all bodies in the body list - set their tag values
        for (unsigned int i=0; i<nb; i++) body[i]->tag = i;
    }

    if (ThrsafeExchange(&callContext->m_gravityTaken, 1) == 0)
    {
        dxWorld *world = callContext->m_stepperCallContext->m_world;

        // add the gravity force to all bodies
        // since gravity does normally have only one component it's more efficient
        // to run three loops for each individual component
        dxBody *const *const bodyend = body + nb;
        dReal gravity_x = world->gravity[0];
        if (gravity_x) {
            for (dxBody *const *bodycurr = body; bodycurr != bodyend; ++bodycurr) {
                dxBody *b = *bodycurr;
                if ((b->flags & dxBodyNoGravity) == 0) {
                    b->facc[0] += b->mass.mass * gravity_x;
                }
            }
        }
        dReal gravity_y = world->gravity[1];
        if (gravity_y) {
            for (dxBody *const *bodycurr = body; bodycurr != bodyend; ++bodycurr) {
                dxBody *b = *bodycurr;
                if ((b->flags & dxBodyNoGravity) == 0) {
                    b->facc[1] += b->mass.mass * gravity_y;
                }
            }
        }
        dReal gravity_z = world->gravity[2];
        if (gravity_z) {
            for (dxBody *const *bodycurr = body; bodycurr != bodyend; ++bodycurr) {
                dxBody *b = *bodycurr;
                if ((b->flags & dxBodyNoGravity) == 0) {
                    b->facc[2] += b->mass.mass * gravity_z;
                }
            }
        }
    }

    // for all bodies, compute the inertia tensor and its inverse in the global
    // frame, and compute the rotational force and add it to the torque
    // accumulator. I and invI are a vertical stack of 3x4 matrices, one per body.
    {
        dReal *invIrow = callContext->m_invI;
        unsigned int bodyIndex = ThrsafeIncrementIntUpToLimit(&callContext->m_inertiaBodyIndex, nb);

        for (unsigned int i = 0; i != nb; invIrow += 12, ++i) {
            if (i == bodyIndex) {
                dMatrix3 tmp;
                dxBody *b = body[i];

                // compute inverse inertia tensor in global frame
                dMultiply2_333 (tmp,b->invI,b->posr.R);
                dMultiply0_333 (invIrow,b->posr.R,tmp);

                // Don't apply gyroscopic torques to bodies
                // if not flagged or the body is kinematic
                if ((b->flags & dxBodyGyroscopic)&& (b->invMass>0)) {
                    dMatrix3 I;
                    // compute inertia tensor in global frame
                    dMultiply2_333 (tmp,b->mass.I,b->posr.R);
                    dMultiply0_333 (I,b->posr.R,tmp);
                    // compute rotational force
#if 0
                    // Explicit computation
                    dMultiply0_331 (tmp,I,b->avel);
                    dSubtractVectorCross3(b->tacc,b->avel,tmp);
#else
                    // Do the implicit computation based on 
                    //"Stabilizing Gyroscopic Forces in Rigid Multibody Simulations"
                    // (Lacoursière 2006)
                    dReal h = callContext->m_stepperCallContext->m_stepSize; // Step size
                    dVector3 L; // Compute angular momentum
                    dMultiply0_331(L,I,b->avel);
                    
                    // Compute a new effective 'inertia tensor'
                    // for the implicit step: the cross-product 
                    // matrix of the angular momentum plus the
                    // old tensor scaled by the timestep.  
                    // Itild may not be symmetric pos-definite, 
                    // but we can still use it to compute implicit
                    // gyroscopic torques.
                    dMatrix3 Itild={0};  
                    dSetCrossMatrixMinus(Itild,L,4);
                    for (int ii=0;ii<12;++ii) {
                      Itild[ii]=Itild[ii]*h+I[ii];
                    }

                    // Scale momentum by inverse time to get 
                    // a sort of "torque"
                    dScaleVector3(L,dRecip(h)); 
                    // Invert the pseudo-tensor
                    dMatrix3 itInv;
                    // This is a closed-form inversion.
                    // It's probably not numerically stable
                    // when dealing with small masses with
                    // a large asymmetry.
                    // An LU decomposition might be better.
                    if (dInvertMatrix3(itInv,Itild)!=0) {
                        // "Divide" the original tensor
                        // by the pseudo-tensor (on the right)
                        dMultiply0_333(Itild,I,itInv);
                        // Subtract an identity matrix
                        Itild[0]-=1; Itild[5]-=1; Itild[10]-=1;

                        // This new inertia matrix rotates the 
                        // momentum to get a new set of torques
                        // that will work correctly when applied
                        // to the old inertia matrix as explicit
                        // torques with a semi-implicit update
                        // step.
                        dVector3 tau0;
                        dMultiply0_331(tau0,Itild,L);
                        
                        // Add the gyro torques to the torque 
                        // accumulator
                        for (int ii=0;ii<3;++ii) {
                          b->tacc[ii]+=tau0[ii];
                        }
                    }
#endif
                }

                bodyIndex = ThrsafeIncrementIntUpToLimit(&callContext->m_inertiaBodyIndex, nb);
            }
        }
    }
}

static 
int dxStepIsland_Stage0_Joints_Callback(void *_callContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee)
{
    (void)callInstanceIndex; // unused
    (void)callThisReleasee; // unused
    dxStepperStage0JointsCallContext *callContext = (dxStepperStage0JointsCallContext *)_callContext;
    dxStepIsland_Stage0_Joints(callContext);
    return 1;
}

static 
void dxStepIsland_Stage0_Joints(dxStepperStage0JointsCallContext *callContext)
{
    dxJoint * const *_joint = callContext->m_stepperCallContext->m_islandJointsStart;
    dJointWithInfo1 *jointinfos = callContext->m_jointinfos;
    unsigned int _nj = callContext->m_stepperCallContext->m_islandJointsCount;

    // get m = total constraint dimension, nub = number of unbounded variables.
    // create constraint offset array and number-of-rows array for all joints.
    // the constraints are re-ordered as follows: the purely unbounded
    // constraints, the mixed unbounded + LCP constraints, and last the purely
    // LCP constraints. this assists the LCP solver to put all unbounded
    // variables at the start for a quick factorization.
    //
    // joints with m=0 are inactive and are removed from the joints array
    // entirely, so that the code that follows does not consider them.
    // also number all active joints in the joint list (set their tag values).
    // inactive joints receive a tag value of -1.

    size_t ji_start, ji_end;
    {
        unsigned int mcurr = 0;
        size_t unb_start, mix_start, mix_end, lcp_end;
        unb_start = mix_start = mix_end = lcp_end = _nj;

        dJointWithInfo1 *jicurr = jointinfos + lcp_end;
        dxJoint *const *const _jend = _joint + _nj;
        dxJoint *const *_jcurr = _joint;
        while (true) {
            // -------------------------------------------------------------------------
            // Switch to growing array forward
            {
                bool fwd_end_reached = false;
                dJointWithInfo1 *jimixend = jointinfos + mix_end;
                while (true) {	// jicurr=dest, _jcurr=src
                    if (_jcurr == _jend) {
                        lcp_end = jicurr - jointinfos;
                        fwd_end_reached = true;
                        break;
                    }
                    dxJoint *j = *_jcurr++;
                    j->getInfo1 (&jicurr->info);
                    dIASSERT (/*jicurr->info.m >= 0 && */jicurr->info.m <= 6 && /*jicurr->info.nub >= 0 && */jicurr->info.nub <= jicurr->info.m);
                    if (jicurr->info.m != 0) {
                        mcurr += jicurr->info.m;
                        if (jicurr->info.nub == 0) { // A lcp info - a correct guess!!!
                            jicurr->joint = j;
                            ++jicurr;
                        } else if (jicurr->info.nub < jicurr->info.m) { // A mixed case
                            if (unb_start == mix_start) { // no unbounded infos yet - just move to opposite side of mixed-s
                                unb_start = mix_start = mix_start - 1;
                                dJointWithInfo1 *jimixstart = jointinfos + mix_start;
                                jimixstart->info = jicurr->info;
                                jimixstart->joint = j;
                            } else if (jimixend != jicurr) { // have to swap to the tail of mixed-s
                                dxJoint::Info1 tmp_info = jicurr->info;
                                *jicurr = *jimixend;
                                jimixend->info = tmp_info;
                                jimixend->joint = j;
                                ++jimixend; ++jicurr;
                            } else { // no need to swap as there are no LCP info-s yet
                                jicurr->joint = j;
                                jimixend = jicurr = jicurr + 1;
                            }
                        } else { // A purely unbounded case -- break out and proceed growing in opposite direction
                            unb_start = unb_start - 1;
                            dJointWithInfo1 *jiunbstart = jointinfos + unb_start;
                            jiunbstart->info = jicurr->info;
                            jiunbstart->joint = j;
                            lcp_end = jicurr - jointinfos;
                            mix_end = jimixend - jointinfos;
                            jicurr = jiunbstart - 1;
                            break;
                        }
                    } else {
                        j->tag = -1;
                    }
                }
                if (fwd_end_reached) {
                    break;
                }
            }
            // -------------------------------------------------------------------------
            // Switch to growing array backward
            {
                bool bkw_end_reached = false;
                dJointWithInfo1 *jimixstart = jointinfos + mix_start - 1;
                while (true) {	// jicurr=dest, _jcurr=src
                    if (_jcurr == _jend) {
                        unb_start = (jicurr + 1) - jointinfos;
                        mix_start = (jimixstart + 1) - jointinfos;
                        bkw_end_reached = true;
                        break;
                    }
                    dxJoint *j = *_jcurr++;
                    j->getInfo1 (&jicurr->info);
                    dIASSERT (/*jicurr->info.m >= 0 && */jicurr->info.m <= 6 && /*jicurr->info.nub >= 0 && */jicurr->info.nub <= jicurr->info.m);
                    if (jicurr->info.m != 0) {
                        mcurr += jicurr->info.m;
                        if (jicurr->info.nub == jicurr->info.m) { // An unbounded info - a correct guess!!!
                            jicurr->joint = j;
                            --jicurr;
                        } else if (jicurr->info.nub != 0) { // A mixed case
                            if (mix_end == lcp_end) { // no lcp infos yet - just move to opposite side of mixed-s
                                dJointWithInfo1 *jimixend = jointinfos + mix_end;
                                lcp_end = mix_end = mix_end + 1;
                                jimixend->info = jicurr->info;
                                jimixend->joint = j;
                            } else if (jimixstart != jicurr) { // have to swap to the head of mixed-s
                                dxJoint::Info1 tmp_info = jicurr->info;
                                *jicurr = *jimixstart;
                                jimixstart->info = tmp_info;
                                jimixstart->joint = j;
                                --jimixstart; --jicurr;
                            } else { // no need to swap as there are no unbounded info-s yet
                                jicurr->joint = j;
                                jimixstart = jicurr = jicurr - 1;
                            }
                        } else { // A purely lcp case -- break out and proceed growing in opposite direction
                            dJointWithInfo1 *jilcpend = jointinfos + lcp_end;
                            lcp_end = lcp_end + 1;
                            jilcpend->info = jicurr->info;
                            jilcpend->joint = j;
                            unb_start = (jicurr + 1) - jointinfos;
                            mix_start = (jimixstart + 1) - jointinfos;
                            jicurr = jilcpend + 1;
                            break;
                        }
                    } else {
                        j->tag = -1;
                    }
                }
                if (bkw_end_reached) {
                    break;
                }
            }
        }

        callContext->m_stage0Outputs->m = mcurr;
        callContext->m_stage0Outputs->nub = (unsigned)(mix_start - unb_start);
        dIASSERT((size_t)(mix_start - unb_start) <= (size_t)UINT_MAX);
        ji_start = unb_start;
        ji_end = lcp_end;
    }

    {
        const dJointWithInfo1 *jicurr = jointinfos + ji_start;
        const dJointWithInfo1 *const jiend = jointinfos + ji_end;
        for (unsigned int i = 0; jicurr != jiend; i++, ++jicurr) {
            jicurr->joint->tag = i;
        }
    }

    callContext->m_stage0Outputs->ji_start = ji_start;
    callContext->m_stage0Outputs->ji_end = ji_end;
}

static 
int dxStepIsland_Stage1_Callback(void *_stage1CallContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee)
{
    (void)callInstanceIndex; // unused
    (void)callThisReleasee; // unused
    dxStepperStage1CallContext *stage1CallContext = (dxStepperStage1CallContext *)_stage1CallContext;
    dxStepIsland_Stage1(stage1CallContext);
    return 1;
}

static 
void dxStepIsland_Stage1(dxStepperStage1CallContext *stage1CallContext)
{
    const dxStepperProcessingCallContext *callContext = stage1CallContext->m_stepperCallContext;
    dJointWithInfo1 *_jointinfos = stage1CallContext->m_jointinfos;
    dReal *invI = stage1CallContext->m_invI;
    size_t ji_start = stage1CallContext->m_stage0Outputs.ji_start;
    size_t ji_end = stage1CallContext->m_stage0Outputs.ji_end;
    unsigned int m = stage1CallContext->m_stage0Outputs.m;
    unsigned int nub = stage1CallContext->m_stage0Outputs.nub;

    dxWorldProcessMemArena *memarena = callContext->m_stepperArena;
    {
        memarena->RestoreState(stage1CallContext->m_stageMemArenaState);
        stage1CallContext = NULL; // WARNING! _stage1CallContext is not valid after this point!
        dIVERIFY(stage1CallContext == NULL); // To suppress compiler warnings about unused variable assignment

        unsigned int _nj = callContext->m_islandJointsCount;
        const size_t ji_reserve_count = 2 * (size_t)_nj;
        memarena->ShrinkArray<dJointWithInfo1>(_jointinfos, ji_reserve_count, ji_end);
    }

    dxWorld *world = callContext->m_world;
    dJointWithInfo1 *jointinfos = _jointinfos + ji_start;
    unsigned int nj = (unsigned int)(ji_end - ji_start);
    dIASSERT((size_t)(ji_end - ji_start) <= (size_t)UINT_MAX);

    unsigned int *mindex = NULL;
    dReal *lo = NULL, *hi = NULL, *J = NULL, *A = NULL, *rhs = NULL;
    int *findex = NULL;

    // if there are constraints, compute cforce
    if (m > 0) {
        mindex = memarena->AllocateArray<unsigned int>((size_t)(nj + 1));
        {
            unsigned int *mcurr = mindex;
            unsigned int moffs = 0;
            mcurr[0] = moffs;
            mcurr += 1;

            const dJointWithInfo1 *const jiend = jointinfos + nj;
            for (const dJointWithInfo1 *jicurr = jointinfos; jicurr != jiend; ++jicurr) {
                //dxJoint *joint = jicurr->joint;
                moffs += jicurr->info.m;
                mcurr[0] = moffs;
                mcurr += 1;
            }
        }

        findex = memarena->AllocateArray<int>(m);
        lo = memarena->AllocateArray<dReal>(m);
        hi = memarena->AllocateArray<dReal>(m);
        J = memarena->AllocateArray<dReal>(2 * 8 * (size_t)m);
        A = memarena->AllocateArray<dReal>(m * (size_t)dPAD(m));
        rhs = memarena->AllocateArray<dReal>(m);
    }

    dxStepperLocalContext *localContext = (dxStepperLocalContext *)memarena->AllocateBlock(sizeof(dxStepperLocalContext));
    localContext->Initialize(invI, jointinfos, nj, m, nub, mindex, findex, lo, hi, J, A, rhs);

    void *stage1MemarenaState = memarena->SaveState();
    dxStepperStage3CallContext *stage3CallContext = (dxStepperStage3CallContext*)memarena->AllocateBlock(sizeof(dxStepperStage3CallContext));
    stage3CallContext->Initialize(callContext, localContext, stage1MemarenaState);

    if (m > 0) {
        // create a constraint equation right hand side vector `c', a constraint
        // force mixing vector `cfm', and LCP low and high bound vectors, and an
        // 'findex' vector.
        dReal *JinvM = memarena->AllocateArray<dReal>(2 * 8 * (size_t)m);
        const unsigned int nb = callContext->m_islandBodiesCount;
        size_t cfm_elem = (size_t)m, rhs_tmp_elem = (size_t)nb*8;
        dReal *cfm = memarena->AllocateArray<dReal>(dMAX(cfm_elem, rhs_tmp_elem));
        // dReal *rhs_tmp = cfm; // Reuse the same memory since rhs calculations start after cfm is not needed anymore

        dxStepperStage2CallContext *stage2CallContext = (dxStepperStage2CallContext *)memarena->AllocateBlock(sizeof(dxStepperStage2CallContext));
        stage2CallContext->Initialize(callContext, localContext, JinvM, cfm);

        const unsigned allowedThreads = callContext->m_stepperAllowedThreads;
        dIASSERT(allowedThreads != 0);

        if (allowedThreads == 1)
        {
            dxStepIsland_Stage2a(stage2CallContext);
            dxStepIsland_Stage2b(stage2CallContext);
            dxStepIsland_Stage2c(stage2CallContext);
            dxStepIsland_Stage3(stage3CallContext);
        }
        else
        {
            dCallReleaseeID stage3CallReleasee;
            world->PostThreadedCallForUnawareReleasee(NULL, &stage3CallReleasee, 1, callContext->m_finalReleasee, 
                NULL, &dxStepIsland_Stage3_Callback, stage3CallContext, 0, "StepIsland Stage3");

            dCallReleaseeID stage2bSyncReleasee;
            world->PostThreadedCall(NULL, &stage2bSyncReleasee, 1, stage3CallReleasee, 
                NULL, &dxStepIsland_Stage2bSync_Callback, stage2CallContext, 0, "StepIsland Stage2b Sync");

            dCallReleaseeID stage2aSyncReleasee;
            world->PostThreadedCall(NULL, &stage2aSyncReleasee, allowedThreads, stage2bSyncReleasee, 
                NULL, &dxStepIsland_Stage2aSync_Callback, stage2CallContext, 0, "StepIsland Stage2a Sync");

            world->PostThreadedCallsGroup(NULL, allowedThreads, stage2aSyncReleasee, &dxStepIsland_Stage2a_Callback, stage2CallContext, "StepIsland Stage2a");
        }
    }
    else {
        dxStepIsland_Stage3(stage3CallContext);
    }
}


static 
int dxStepIsland_Stage2a_Callback(void *_stage2CallContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee)
{
    (void)callInstanceIndex; // unused
    (void)callThisReleasee; // unused
    dxStepperStage2CallContext *stage2CallContext = (dxStepperStage2CallContext *)_stage2CallContext;
    dxStepIsland_Stage2a(stage2CallContext);
    return 1;
}

static 
void dxStepIsland_Stage2a(dxStepperStage2CallContext *stage2CallContext)
{
    const dxStepperProcessingCallContext *callContext = stage2CallContext->m_stepperCallContext;
    const dxStepperLocalContext *localContext = stage2CallContext->m_localContext;
    dJointWithInfo1 *jointinfos = localContext->m_jointinfos;
    unsigned int nj = localContext->m_nj;
    const unsigned int *mindex = localContext->m_mindex;

    const dReal stepsizeRecip = dRecip(callContext->m_stepSize);
    dxWorld *world = callContext->m_world;

    {
        int *findex = localContext->m_findex;
        dReal *J = localContext->m_J;
        dReal *cfm = stage2CallContext->m_rhs_tmp_or_cfm;
        dReal *lo = localContext->m_lo;
        dReal *hi = localContext->m_hi;
        dReal *rhs = localContext->m_rhs;

        IFTIMING(dTimerNow ("create J"));
        // get jacobian data from constraints. a (2*m)x8 matrix will be created
        // to store the two jacobian blocks from each constraint. it has this
        // format:
        //
        //   l l l 0 a a a 0  \    .
        //   l l l 0 a a a 0   }-- jacobian body 1 block for joint 0 (3 rows)
        //   l l l 0 a a a 0  /
        //   l l l 0 a a a 0  \    .
        //   l l l 0 a a a 0   }-- jacobian body 2 block for joint 0 (3 rows)
        //   l l l 0 a a a 0  /
        //   l l l 0 a a a 0  }--- jacobian body 1 block for joint 1 (1 row)
        //   l l l 0 a a a 0  }--- jacobian body 2 block for joint 1 (1 row)
        //   etc...
        //
        //   (lll) = linear jacobian data
        //   (aaa) = angular jacobian data
        //

        const dReal worldERP = world->global_erp;

        dxJoint::Info2Descr Jinfo;
        Jinfo.rowskip = 8;

        unsigned ji;
        while ((ji = ThrsafeIncrementIntUpToLimit(&stage2CallContext->m_ji_J, nj)) != nj) {
            const unsigned ofsi = mindex[ji];
            const unsigned int infom = mindex[ji + 1] - ofsi;

            dReal *const J1row = J + 2*8*(size_t)ofsi;
            Jinfo.J1l = J1row;
            Jinfo.J1a = J1row + 4;
            dReal *const J2row = J1row + 8*infom;
            Jinfo.J2l = J2row;
            Jinfo.J2a = J2row + 4;
			dSetZero (J1row, 2*8*infom);
            Jinfo.c = rhs + ofsi;
			dSetZero (Jinfo.c, infom);
            Jinfo.cfm = cfm + ofsi;
			dSetValue (Jinfo.cfm, infom, world->global_cfm);
            Jinfo.lo = lo + ofsi;
			dSetValue (Jinfo.lo, infom, -dInfinity);
            Jinfo.hi = hi + ofsi;
			dSetValue (Jinfo.hi, infom, dInfinity);
            Jinfo.findex = findex + ofsi;
            dSetValue(Jinfo.findex, infom, -1);

            dxJoint *joint = jointinfos[ji].joint;
            joint->getInfo2(stepsizeRecip, worldERP, &Jinfo);

            dReal *rhs_row = Jinfo.c;
            for (unsigned int i = 0; i != infom; ++i) {
                rhs_row[i] *= stepsizeRecip;
            }

            // adjust returned findex values for global index numbering
            int *findex_row = Jinfo.findex;
            for (unsigned int j = infom; j != 0; ) {
                --j;
                int fival = findex_row[j];
                if (fival != -1) 
                    findex_row[j] = fival + ofsi;
            }
        }
    }
}

static 
int dxStepIsland_Stage2aSync_Callback(void *_stage2CallContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee)
{
    (void)callInstanceIndex; // unused
    dxStepperStage2CallContext *stage2CallContext = (dxStepperStage2CallContext *)_stage2CallContext;
    const dxStepperProcessingCallContext *callContext = stage2CallContext->m_stepperCallContext;
    dxWorld *world = callContext->m_world;
    const unsigned allowedThreads = callContext->m_stepperAllowedThreads;

    world->AlterThreadedCallDependenciesCount(callThisReleasee, allowedThreads);
    world->PostThreadedCallsGroup(NULL, allowedThreads, callThisReleasee, &dxStepIsland_Stage2b_Callback, stage2CallContext, "StepIsland Stage2b");

    return 1;
}

static 
int dxStepIsland_Stage2b_Callback(void *_stage2CallContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee)
{
    (void)callInstanceIndex; // unused
    (void)callThisReleasee; // unused
    dxStepperStage2CallContext *stage2CallContext = (dxStepperStage2CallContext *)_stage2CallContext;
    dxStepIsland_Stage2b(stage2CallContext);
    return 1;
}

static 
void dxStepIsland_Stage2b(dxStepperStage2CallContext *stage2CallContext)
{
    const dxStepperProcessingCallContext *callContext = stage2CallContext->m_stepperCallContext;
    const dxStepperLocalContext *localContext = stage2CallContext->m_localContext;
    dJointWithInfo1 *jointinfos = localContext->m_jointinfos;
    unsigned int nj = localContext->m_nj;
    const unsigned int *mindex = localContext->m_mindex;

    {
        // Warning!!!
        // This code depends on cfm elements and therefore must be in different sub-stage 
        // from Jacobian construction in Stage2a to ensure proper synchronization 
        // and avoid accessing numbers being modified.
        // Warning!!!
        const dReal stepsizeRecip = dRecip(callContext->m_stepSize);

        dReal *A = localContext->m_A;
        const dReal *cfm = stage2CallContext->m_rhs_tmp_or_cfm;
        const unsigned m = localContext->m_m;

        const unsigned int mskip = dPAD(m);

        unsigned ji;
        while ((ji = ThrsafeIncrementIntUpToLimit(&stage2CallContext->m_ji_Ainit, nj)) != nj) {
            const unsigned ofsi = mindex[ji];
            const unsigned int infom = mindex[ji + 1] - ofsi;

            dReal *Arow = A + (size_t)mskip*ofsi;
            dSetZero(Arow, (size_t)mskip*infom);
            dReal *Adiag = Arow + ofsi;
            const dReal *cfm_block = cfm + ofsi;
            for (unsigned int i = 0; i != infom; Adiag += mskip, ++i) {
                Adiag[i] = cfm_block[i] * stepsizeRecip;
            }
        }
    }

    {
        // Warning!!!
        // This code depends on J elements and therefore must be in different sub-stage 
        // from Jacobian construction in Stage2a to ensure proper synchronization 
        // and avoid accessing numbers being modified.
        // Warning!!!
        const dReal *invI = localContext->m_invI;
        const dReal *J = localContext->m_J;
        dReal *JinvM = stage2CallContext->m_JinvM;

        IFTIMING(dTimerNow ("compute JinvM"));
        // compute A = J*invM*J'. first compute JinvM = J*invM. this has the same
        // format as J so we just go through the constraints in J multiplying by
        // the appropriate scalars and matrices.
        unsigned ji;
        while ((ji = ThrsafeIncrementIntUpToLimit(&stage2CallContext->m_ji_JinvM, nj)) != nj) {
            const unsigned ofsi = mindex[ji];
            const unsigned int infom = mindex[ji + 1] - ofsi;

            dReal *Jdst = JinvM + 2*8*(size_t)ofsi;
            dSetZero(Jdst, 2*8*infom);

            const dReal *Jsrc = J + 2*8*(size_t)ofsi;
            dxJoint *joint = jointinfos[ji].joint;

            dxBody *jb0 = joint->node[0].body;
            if (true || jb0 != NULL) { // -- always true
                dReal body_invMass0 = jb0->invMass;
                const dReal *body_invI0 = invI + (size_t)(unsigned int)jb0->tag*12;
                for (unsigned int j=infom; j>0;) {
                    --j;
                    for (unsigned int k=0; k<3; ++k) Jdst[k] = Jsrc[k] * body_invMass0;
                    dMultiply0_133(Jdst+4, Jsrc+4, body_invI0);
                    Jsrc += 8;
                    Jdst += 8;
                }
            }

            dxBody *jb1 = joint->node[1].body;
            if (jb1 != NULL) {
                dReal body_invMass1 = jb1->invMass;
                const dReal *body_invI1 = invI + (size_t)(unsigned int)jb1->tag*12;
                for (unsigned int j=infom; j>0; ) {
                    --j;
                    for (unsigned int k=0; k<3; ++k) Jdst[k] = Jsrc[k] * body_invMass1;
                    dMultiply0_133 (Jdst+4,Jsrc+4,body_invI1);
                    Jsrc += 8;
                    Jdst += 8;
                }
            }
        }
    }

    {
        // Warning!!!
        // This code reads facc/tacc fields of body objects which (the fields)
        // may be modified by dxJoint::getInfo2(). Therefore the code must be
        // in different sub-stage from Jacobian construction in Stage2a 
        // to ensure proper synchronization and avoid accessing numbers being modified.
        // Warning!!!
        dxBody * const *const body = callContext->m_islandBodiesStart;
        const unsigned int nb = callContext->m_islandBodiesCount;
        const dReal *invI = localContext->m_invI;
        dReal *rhs_tmp = stage2CallContext->m_rhs_tmp_or_cfm;

        // compute the right hand side `rhs'
        IFTIMING(dTimerNow ("compute rhs_tmp"));
        const dReal stepsizeRecip = dRecip(callContext->m_stepSize);

        // put v/h + invM*fe into rhs_tmp
        unsigned bi;
        while ((bi = ThrsafeIncrementIntUpToLimit(&stage2CallContext->m_bi_rhs_tmp, nb)) != nb) {
            dReal *tmp1curr = rhs_tmp + (size_t)bi * 8;
            const dReal *invIrow = invI + (size_t)bi * 12;
            dxBody *b = body[bi];
            // dSetZero(tmp1curr, 8); -- not needed
            for (unsigned int j=0; j<3; ++j) tmp1curr[j] = b->facc[j]*b->invMass + b->lvel[j]*stepsizeRecip;
            dMultiply0_331 (tmp1curr+4, invIrow, b->tacc);
            for (unsigned int k=0; k<3; ++k) tmp1curr[4+k] += b->avel[k]*stepsizeRecip;
        }
    }
}

static 
int dxStepIsland_Stage2bSync_Callback(void *_stage2CallContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee)
{
    (void)callInstanceIndex; // unused
    dxStepperStage2CallContext *stage2CallContext = (dxStepperStage2CallContext *)_stage2CallContext;
    const dxStepperProcessingCallContext *callContext = stage2CallContext->m_stepperCallContext;
    dxWorld *world = callContext->m_world;
    const unsigned allowedThreads = callContext->m_stepperAllowedThreads;

    world->AlterThreadedCallDependenciesCount(callThisReleasee, allowedThreads);
    world->PostThreadedCallsGroup(NULL, allowedThreads, callThisReleasee, &dxStepIsland_Stage2c_Callback, stage2CallContext, "StepIsland Stage2c");

    return 1;
}


static 
int dxStepIsland_Stage2c_Callback(void *_stage2CallContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee)
{
    (void)callInstanceIndex; // unused
    (void)callThisReleasee; // unused
    dxStepperStage2CallContext *stage2CallContext = (dxStepperStage2CallContext *)_stage2CallContext;
    dxStepIsland_Stage2c(stage2CallContext);
    return 1;
}

static 
void dxStepIsland_Stage2c(dxStepperStage2CallContext *stage2CallContext)
{
    //const dxStepperProcessingCallContext *callContext = stage2CallContext->m_stepperCallContext;
    const dxStepperLocalContext *localContext = stage2CallContext->m_localContext;
    dJointWithInfo1 *jointinfos = localContext->m_jointinfos;
    unsigned int nj = localContext->m_nj;
    const unsigned int *mindex = localContext->m_mindex;

    {
        // Warning!!!
        // This code depends on A elements and JinvM elements and therefore 
        // must be in different sub-stage from A initialization and JinvM calculation in Stage2b 
        // to ensure proper synchronization and avoid accessing numbers being modified.
        // Warning!!!
        dReal *A = localContext->m_A;
        const dReal *JinvM = stage2CallContext->m_JinvM;
        const dReal *J = localContext->m_J;
        const unsigned m = localContext->m_m;

        // now compute A = JinvM * J'. A's rows and columns are grouped by joint,
        // i.e. in the same way as the rows of J. block (i,j) of A is only nonzero
        // if joints i and j have at least one body in common. 
        const unsigned int mskip = dPAD(m);

        unsigned ji;
        while ((ji = ThrsafeIncrementIntUpToLimit(&stage2CallContext->m_ji_Aaddjb, nj)) != nj) {
            const unsigned ofsi = mindex[ji];
            const unsigned int infom = mindex[ji + 1] - ofsi;

            dReal *Arow = A + (size_t)mskip*ofsi;
            const dReal *JinvMrow = JinvM + 2*8*(size_t)ofsi;
            dxJoint *joint = jointinfos[ji].joint;

            dxBody *jb0 = joint->node[0].body;
            if (true || jb0 != NULL) { // -- always true
                // compute diagonal block of A
                MultiplyAdd2_p8r (Arow + ofsi, JinvMrow, 
                    J + 2*8*(size_t)ofsi, infom, infom, mskip);

                for (dxJointNode *n0=(ji != 0 ? jb0->firstjoint : NULL); n0; n0=n0->next) {
                    // if joint was tagged as -1 then it is an inactive (m=0 or disabled)
                    // joint that should not be considered
                    int j0 = n0->joint->tag;
                    if (j0 != -1 && (unsigned)j0 < ji) {
                        const unsigned int jiother_ofsi = mindex[j0];
                        const unsigned int jiother_infom = mindex[j0 + 1] - jiother_ofsi;
                        const dJointWithInfo1 *jiother = jointinfos + j0;
                        unsigned int ofsother = (jiother->joint->node[1].body == jb0) ? 8*jiother_infom : 0;
                        // set block of A
                        MultiplyAdd2_p8r (Arow + jiother_ofsi, JinvMrow, 
                            J + 2*8*(size_t)jiother_ofsi + ofsother, infom, jiother_infom, mskip);
                    }
                }
            }

            dxBody *jb1 = joint->node[1].body;
            dIASSERT(jb1 != jb0);
            if (jb1 != NULL) {
                // compute diagonal block of A
                MultiplyAdd2_p8r (Arow + ofsi, JinvMrow + 8*infom, 
                    J + 2*8*(size_t)ofsi + 8*infom, infom, infom, mskip);

                for (dxJointNode *n1=(ji != 0 ? jb1->firstjoint : NULL); n1; n1=n1->next) {
                    // if joint was tagged as -1 then it is an inactive (m=0 or disabled)
                    // joint that should not be considered
                    int j1 = n1->joint->tag;
                    if (j1 != -1 && (unsigned)j1 < ji) {
                        const unsigned int jiother_ofsi = mindex[j1];
                        const unsigned int jiother_infom = mindex[j1 + 1] - jiother_ofsi;
                        const dJointWithInfo1 *jiother = jointinfos + j1;
                        unsigned int ofsother = (jiother->joint->node[1].body == jb1) ? 8*jiother_infom : 0;
                        // set block of A
                        MultiplyAdd2_p8r (Arow + jiother_ofsi, JinvMrow + 8*infom, 
                            J + 2*8*(size_t)jiother_ofsi + ofsother, infom, jiother_infom, mskip);
                    }
                }
            }
        }
    }

    {
        // Warning!!!
        // This code depends on rhs_tmp elements and therefore must be in 
        // different sub-stage from rhs_tmp calculation in Stage2b to ensure 
        // proper synchronization and avoid accessing numbers being modified.
        // Warning!!!
        const dReal *J = localContext->m_J;
        const dReal *rhs_tmp = stage2CallContext->m_rhs_tmp_or_cfm;
        dReal *rhs = localContext->m_rhs;

        // compute the right hand side `rhs'
        IFTIMING(dTimerNow ("compute rhs"));

        // put J*rhs_tmp into rhs
        unsigned ji;
        while ((ji = ThrsafeIncrementIntUpToLimit(&stage2CallContext->m_ji_rhs, nj)) != nj) {
            const unsigned ofsi = mindex[ji];
            const unsigned int infom = mindex[ji + 1] - ofsi;

            dReal *rhscurr = rhs + ofsi;
            const dReal *Jrow = J + 2*8*(size_t)ofsi;
            
            dxJoint *joint = jointinfos[ji].joint;

            dxBody *jb0 = joint->node[0].body;
            if (true || jb0 != NULL) { // -- always true
                MultiplySub0_p81 (rhscurr, Jrow, rhs_tmp + 8*(size_t)(unsigned)jb0->tag, infom);
            }

            dxBody *jb1 = joint->node[1].body;
            if (jb1 != NULL) {
                MultiplySub0_p81 (rhscurr, Jrow + 8*infom, rhs_tmp + 8*(size_t)(unsigned)jb1->tag, infom);
            }
        }
    }
}


static 
int dxStepIsland_Stage3_Callback(void *_stage3CallContext, dcallindex_t callInstanceIndex, dCallReleaseeID callThisReleasee)
{
    (void)callInstanceIndex; // unused
    (void)callThisReleasee; // unused
    dxStepperStage3CallContext *stage3CallContext = (dxStepperStage3CallContext *)_stage3CallContext;
    dxStepIsland_Stage3(stage3CallContext);
    return 1;
}

static 
void dxStepIsland_Stage3(dxStepperStage3CallContext *stage3CallContext)
{
    const dxStepperProcessingCallContext *callContext = stage3CallContext->m_stepperCallContext;
    const dxStepperLocalContext *localContext = stage3CallContext->m_localContext;

    dxWorldProcessMemArena *memarena = callContext->m_stepperArena;
    memarena->RestoreState(stage3CallContext->m_stage1MemArenaState);
    stage3CallContext = NULL; // WARNING! stage3CallContext is not valid after this point!
    dIVERIFY(stage3CallContext == NULL); // To suppress unused variable assignment warnings

    dReal *invI = localContext->m_invI;
    dJointWithInfo1 *jointinfos = localContext->m_jointinfos;
    unsigned int nj = localContext->m_nj;
    unsigned int m = localContext->m_m;
    unsigned int nub = localContext->m_nub;
    //const unsigned int *mindex = localContext->m_mindex;
    int *findex = localContext->m_findex;
    dReal *lo = localContext->m_lo;
    dReal *hi = localContext->m_hi;
    dReal *J = localContext->m_J;
    dReal *A = localContext->m_A;
    dReal *rhs = localContext->m_rhs;

    //dxWorld *world = callContext->m_world;
    dxBody * const *body = callContext->m_islandBodiesStart;
    unsigned int nb = callContext->m_islandBodiesCount;

    dReal *lambda = NULL;

    if (m > 0) {
        lambda = memarena->AllocateArray<dReal>(m);

        BEGIN_STATE_SAVE(memarena, lcpstate) {
            IFTIMING(dTimerNow ("solving LCP problem"));

            // solve the LCP problem and get lambda.
            // this will destroy A but that's OK
            dSolveLCP (memarena, m, A, lambda, rhs, NULL, nub, lo, hi, findex);

        } END_STATE_SAVE(memarena, lcpstate);
    }

    // this will be set to the force due to the constraints
    dReal *cforce = memarena->AllocateArray<dReal>((size_t)nb * 8);
    dSetZero (cforce,(size_t)nb*8);

    if (m > 0) {
        {
            IFTIMING(dTimerNow ("compute constraint force"));

            // compute the constraint force `cforce'
            // compute cforce = J'*lambda
            unsigned ofsi = 0;
            const dJointWithInfo1 *jicurr = jointinfos;
            const dJointWithInfo1 *const jiend = jicurr + nj;
            for (; jicurr != jiend; ++jicurr) {
                const unsigned int infom = jicurr->info.m;
                dxJoint *joint = jicurr->joint;

                const dReal *JJ = J + 2*8*(size_t)ofsi;
                const dReal *lambdarow = lambda + ofsi;

                dJointFeedback *fb = joint->feedback;

                if (fb) {
                    // the user has requested feedback on the amount of force that this
                    // joint is applying to the bodies. we use a slightly slower
                    // computation that splits out the force components and puts them
                    // in the feedback structure.
                    dReal data[8];
                    Multiply1_8q1 (data, JJ, lambdarow, infom);

                    dxBody* b1 = joint->node[0].body;
                    dReal *cf1 = cforce + 8*(size_t)(unsigned)b1->tag;
                    cf1[0] += (fb->f1[0] = data[0]);
                    cf1[1] += (fb->f1[1] = data[1]);
                    cf1[2] += (fb->f1[2] = data[2]);
                    cf1[4] += (fb->t1[0] = data[4]);
                    cf1[5] += (fb->t1[1] = data[5]);
                    cf1[6] += (fb->t1[2] = data[6]);

                    dxBody* b2 = joint->node[1].body;
                    if (b2){
                        Multiply1_8q1 (data, JJ + 8*infom, lambdarow, infom);

                        dReal *cf2 = cforce + 8*(size_t)(unsigned)b2->tag;
                        cf2[0] += (fb->f2[0] = data[0]);
                        cf2[1] += (fb->f2[1] = data[1]);
                        cf2[2] += (fb->f2[2] = data[2]);
                        cf2[4] += (fb->t2[0] = data[4]);
                        cf2[5] += (fb->t2[1] = data[5]);
                        cf2[6] += (fb->t2[2] = data[6]);
                    }
                }
                else {
                    // no feedback is required, let's compute cforce the faster way
                    dxBody* b1 = joint->node[0].body;
                    dReal *cf1 = cforce + 8*(size_t)(unsigned)b1->tag;
                    MultiplyAdd1_8q1 (cf1, JJ, lambdarow, infom);

                    dxBody* b2 = joint->node[1].body;
                    if (b2) {
                        dReal *cf2 = cforce + 8*(size_t)(unsigned)b2->tag;
                        MultiplyAdd1_8q1 (cf2, JJ + 8*infom, lambdarow, infom);
                    }
                }

                ofsi += infom;
            }
        }
    } // if (m > 0)

    {
        // compute the velocity update
        IFTIMING(dTimerNow ("compute velocity update"));

        const dReal stepsize = callContext->m_stepSize;

        // add fe to cforce and multiply cforce by stepsize
        dReal data[4];
        const dReal *invIrow = invI;
        dReal *cforcecurr = cforce;
        dxBody *const *const bodyend = body + nb;
        for (dxBody *const *bodycurr = body; bodycurr != bodyend; invIrow+=12, cforcecurr+=8, ++bodycurr) {
            dxBody *b = *bodycurr;

            dReal body_invMass_mul_stepsize = stepsize * b->invMass;
            for (unsigned int j=0; j<3; ++j) b->lvel[j] += (cforcecurr[j] + b->facc[j]) * body_invMass_mul_stepsize;

            for (unsigned int k=0; k<3; ++k) data[k] = (cforcecurr[4+k] + b->tacc[k]) * stepsize;
            dMultiplyAdd0_331 (b->avel, invIrow, data);
        }
    }

    {
        // update the position and orientation from the new linear/angular velocity
        // (over the given timestep)
        IFTIMING(dTimerNow ("update position"));

        const dReal stepsize = callContext->m_stepSize;

        dxBody *const *const bodyend = body + nb;
        for (dxBody *const *bodycurr = body; bodycurr != bodyend; ++bodycurr) {
            dxBody *b = *bodycurr;
            dxStepBody (b, stepsize);
        }
    }

    {
        IFTIMING(dTimerNow ("tidy up"));

        // zero all force accumulators
        dxBody *const *const bodyend = body + nb;
        for (dxBody *const *bodycurr = body; bodycurr != bodyend; ++bodycurr) {
            dxBody *b = *bodycurr;
            b->facc[0] = 0;
            b->facc[1] = 0;
            b->facc[2] = 0;
            b->facc[3] = 0;
            b->tacc[0] = 0;
            b->tacc[1] = 0;
            b->tacc[2] = 0;
            b->tacc[3] = 0;
        }
    }

    IFTIMING(dTimerEnd());
    if (m > 0) IFTIMING(dTimerReport (stdout,1));
}

//****************************************************************************

/*extern */
size_t dxEstimateStepMemoryRequirements (dxBody * const *body, unsigned int nb, dxJoint * const *_joint, unsigned int _nj)
{
    (void)body; // unused
    unsigned int nj, m;

    {
        unsigned int njcurr = 0, mcurr = 0;
        dxJoint::SureMaxInfo info;
        dxJoint *const *const _jend = _joint + _nj;
        for (dxJoint *const *_jcurr = _joint; _jcurr != _jend; ++_jcurr) {	
            dxJoint *j = *_jcurr;
            j->getSureMaxInfo (&info);

            unsigned int jm = info.max_m;
            if (jm > 0) {
                njcurr++;

                mcurr += jm;
            }
        }
        nj = njcurr; m = mcurr;
    }

    size_t res = 0;

    res += dEFFICIENT_SIZE(sizeof(dReal) * 3 * 4 * nb); // for invI

    {
        size_t sub1_res1 = dEFFICIENT_SIZE(sizeof(dJointWithInfo1) * 2 * _nj); // for initial jointinfos

        // The array can't grow right more than by nj
        size_t sub1_res2 = dEFFICIENT_SIZE(sizeof(dJointWithInfo1) * ((size_t)_nj + (size_t)nj)); // for shrunk jointinfos
        sub1_res2 += dEFFICIENT_SIZE(sizeof(dxStepperLocalContext)); //for dxStepperLocalContext
        if (m > 0) {
            sub1_res2 += dEFFICIENT_SIZE(sizeof(unsigned int) * (nj + 1)); // for mindex
            sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 2 * 8 * m); // for J
            unsigned int mskip = dPAD(m);
            sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * mskip * m); // for A
            sub1_res2 += 3 * dEFFICIENT_SIZE(sizeof(dReal) * m); // for lo, hi, rhs
            sub1_res2 += dEFFICIENT_SIZE(sizeof(int) * m); // for findex
            {
                size_t sub2_res1 = dEFFICIENT_SIZE(sizeof(dxStepperStage3CallContext)); //for dxStepperStage3CallContext
                sub2_res1 += dEFFICIENT_SIZE(sizeof(dReal) * 2 * 8 * m); // for JinvM
                sub2_res1 += dEFFICIENT_SIZE(sizeof(dReal) * dMAX((size_t)m, (size_t)nb * 8)); // for cfm and rhs_tmp
                sub2_res1 += dEFFICIENT_SIZE(sizeof(dxStepperStage2CallContext)); // for dxStepperStage2CallContext

                size_t sub2_res2 = dEFFICIENT_SIZE(sizeof(dReal) * m); // for lambda
                {
                    size_t sub3_res1 = dEstimateSolveLCPMemoryReq(m, false);

                    size_t sub3_res2 = dEFFICIENT_SIZE(sizeof(dReal) * 8 * nb); // for cforce

                    sub2_res2 += dMAX(sub3_res1, sub3_res2);
                }

                sub1_res2 += dMAX(sub2_res1, sub2_res2);
            }
        }
        else {
            sub1_res2 += dEFFICIENT_SIZE(sizeof(dxStepperStage3CallContext)); // for dxStepperStage3CallContext
            sub1_res2 += dEFFICIENT_SIZE(sizeof(dReal) * 8 * nb); // for cforce
        }

        size_t sub1_res12_max = dMAX(sub1_res1, sub1_res2);
        size_t stage01_contexts = dEFFICIENT_SIZE(sizeof(dxStepperStage0BodiesCallContext))
            + dEFFICIENT_SIZE(sizeof(dxStepperStage0JointsCallContext))
            + dEFFICIENT_SIZE(sizeof(dxStepperStage1CallContext));
        res += dMAX(sub1_res12_max, stage01_contexts);
    }

    return res;
}


/*extern */
unsigned dxEstimateStepMaxCallCount(
    unsigned /*activeThreadCount*/, unsigned allowedThreadCount)
{
    unsigned result = 1 // dxStepIsland itself
        + (2 * allowedThreadCount + 2) // (dxStepIsland_Stage2a + dxStepIsland_Stage2b) * allowedThreadCount + 2 * dxStepIsland_Stage2?_Sync
        + 1; // dxStepIsland_Stage3
    return result;
}
