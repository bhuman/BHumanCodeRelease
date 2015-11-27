#include "DmpKickEngine.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Motion/InverseKinematic.h"
#include <limits>
#include <iostream>
#include "Tools/Motion/DynamicMotionPrimitive.h"
#include <functional>
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Tools/Streams/OutStreams.h"

using namespace std;

MAKE_MODULE(DmpKickEngine, motionControl)

DmpKickEngine::DmpKickEngine()
{
  for(int i = 0; i < numOfStates; ++i)
  {
    states[i] = nullptr;
  }
  states[Idle] = std::bind(&DmpKickEngine::stateIdle, this);
  states[InitKick] = std::bind(&DmpKickEngine::stateInitKick, this);
  states[Balance] = std::bind(&DmpKickEngine::stateBalance, this);
  states[IterateModel] = std::bind(&DmpKickEngine::stateIterateModel, this);
  states[InitDynKick] = std::bind(&DmpKickEngine::stateInitDynKick, this);
  states[DynKick] = std::bind(&DmpKickEngine::stateDynKick, this);
  states[Setup] = std::bind(&DmpKickEngine::stateSetup, this);
  states[InitImitatedKick] = std::bind(&DmpKickEngine::stateInitImitatedKick, this);
  states[ImitatedKick] = std::bind(&DmpKickEngine::stateImitatedKick, this);

  for(int i = 0; i < numOfStates; ++i)
  {
    ASSERT(states[i] != nullptr);
  }

  state = Setup;

  oldComHeight = 0.0;
  oldNumZmpPreviews = 0;
  oldR = 0.0;
  oldQe = 0.0;
  oldQx = 0.0;
  oldR0DiagNorm = 0.0;
  oldQlDiagNorm = 0.0;

  targets.push_back(Vector2f(0, -50));
  targets.push_back(Vector2f(0, 0));
 // targets.push_back(Vector2f(25, 0));
 // targets.push_back(Vector2f(-15, -100));
 // targets.push_back(Vector2f(25, -100));
  targets.push_back(Vector2f(0, -50));

  //imitate judys kick
  DynamicMotionPrimitive<3>::Matrixf positions(3, 97);
  positions.row(0) << 0.0f, 0.064780636330682387f, 0.053692151518963123f, 0.022223261833499819f, 0.02787323282618618f, 0.065811704409219948f, 0.098838623579606502f, 0.10801040350306511f, 0.090319357773635922f, 0.050075199299914684f, 0.005656536735271789f, 0.0013107849334073112f, 0.060523888719173552f, 0.13523153552984707f, 0.17141592107090439f, 0.15928109026051432f, 0.13728662880890793f, 0.12092638898901344f, 0.095054656878813718f, 0.058657964117890239f, 0.032674606969810976f, 0.062970331589616554f, 0.22074752256167307f, 0.5640130705723877f, 1.040972754706347f, 1.4005721359311745f, 1.4244619633928319f, 1.1151069657466754f, 0.66204487442873072f, 0.24142464258209675f, -0.10613703304843672f, -0.4097509591038192f, -0.69839691800919412f, -0.96539476896112919f, -1.1712880570078033f, -1.2715952527427992f, -1.2463153014098178f, -1.0963845316347285f, -0.85247833244311044f, -0.57858051081112694f, -0.30638504207460099f, -0.023134423228608821f, 0.2932873791343073f, 0.64294057761705004f, 1.0058635496328259f, 1.371572231635128f, 1.7372944959282377f, 2.0908362925097275f, 2.4026853991707573f, 2.6635762541384582f, 2.8915608290138377f, 3.0763871879201554f, 3.1750424952118537f, 3.1250471564952775f, 2.8747926065874503f, 2.4981023901328014f, 2.2091828769371653f, 2.0443705131940173f, 1.8915491160284261f, 1.6817412702810877f, 1.4072669730967236f, 1.0798093131779205f, 0.70726629762780691f, 0.3412708175022674f, 0.043329631736219426f, -0.21973790719743055f, -0.53421781145277281f, -0.96908997878843817f, -1.570194658864106f, -2.5851402475850138f, -4.4829516599402552f, -7.5721753769746281f, -11.82184161126894f, -16.898115716125758f, -22.528948729495852f, -28.510215558376046f, -34.596151163903379f, -40.568405897208471f, -46.227419984823406f, -51.409883989019932f, -56.101018613901033f, -60.388618700176139f, -64.322777209583776f, -67.709682655423876f, -70.123942895861205f, -71.188944449759219f, -70.826909357041359f, -69.098879428457195f, -65.744848695133314f, -60.224791126530022f, -51.992486169046892f, -40.83393080714238f, -26.910475622508166f, -10.740908638749689f, 7.1295542087189769f, 26.266586212252694f, 46.244469132653279f;
  positions.row(1).setConstant(0);
  positions.row(2) << 0.0f, -0.016912573463817273f, 0.011294297730400554f, 0.063810889643810528f, 0.099711152911641898f, 0.088954741772013138f, 0.031589442785442262f, -0.039351160133447742f, -0.093421680774814192f, -0.11820823059168065f, -0.11798353791171454f, -0.10545345912905112f, -0.091588058681136758f, -0.07811721828131829f, -0.06261264421336224f, -0.043841248372931123f, -0.020716650908969941f, 0.017601877908735908f, 0.072177879751301161f, 0.10695694471153828f, 0.095780046778188424f, 0.050376347876587901f, 0.0057475515734746125f, -0.028755564509055611f, -0.073074310300348749f, -0.12437500037662781f, -0.14628246579452897f, -0.10460789122659392f, -0.03368102862958175f, 0.015099659322855652f, 0.027641059977823117f, 0.014013732667095099f, -0.012852348709126748f, -0.04609536814140431f, -0.07885366684649707f, -0.10116526461844982f, -0.10049996613058367f, -0.065176820559820478f, 0.0038017990184245278f, 0.078772933653298668f, 0.11999310769091351f, 0.10869741129722948f, 0.079987440115014949f, 0.078528475605734296f, 0.11457373267326217f, 0.14465433166638089f, 0.12255606430102153f, 0.031941378577455f, -0.11481027064234968f, -0.27747093782831533f, -0.39925726703903441f, -0.43716480920025258f, -0.31557837225700336f, 0.086105800153056425f, 0.81274384729943039f, 1.7464163811909903f, 2.6401965374299334f, 3.4046881782738283f, 4.0687326565074393f, 4.6199214460353719f, 5.0457458667304129f, 5.3717596840959958f, 5.658099885116723f, 5.9549637984146759f, 6.3204680554284369f, 6.8748900087697393f, 7.7583051090559438f, 9.078830149648951f, 10.884110933342187f, 13.21956846468424f, 16.130529980090806f, 19.604235478068993f, 23.588026321040999f, 28.008995464092447f, 32.807737639745021f, 37.885240977525541f, 43.078302825227468f, 48.304276983097331f, 53.522682559185114f, 58.632756388543022f, 63.45751638089169f, 67.789519885654286f, 71.492467681579953f, 74.50784979852169f, 76.884059196853855f, 78.856118768961736f, 80.688280380022249f, 82.268253449196465f, 82.688768658683898f, 80.918983259000839f, 76.499718023453383f, 69.910880575962324f, 62.129799246546554f, 54.223727502753249f, 46.991750497208791f, 41.054999079704118f, 36.966492702571351f;

  DynamicMotionPrimitive<3>::Matrixf velocities(3, 97);
  velocities.row(0) << 6.417520982291899f, 2.6595177855187337f, -2.107982101262314f, -1.2788810193618485f, 2.1590536976758568f, 3.5151081401226882f, 2.090215936424106f, -0.4219823249686362f, -2.869687684828946f, -4.193578986012429f, -2.415433608808309f, 2.7177286496699002f, 6.633457739823651f, 5.492782910880125f, 1.1912396268461347f, -1.6905163456876748f, -1.8998123059715388f, -2.091864030182237f, -3.0843238487565694f, -3.0898529394178924f, 0.21360324859952776f, 9.315761239596906f, 24.81800482811857f, 40.6279787884745f, 41.4370378168371f, 18.995259869517476f, -14.1398635698864f, -37.76458478046482f, -43.275853390395014f, -38.05013186569147f, -32.25449241995658f, -29.33623729244873f, -27.522543852745258f, -23.423579782174095f, -15.166939850867763f, -3.7163027600997873f, 8.678661886661445f, 19.507812481528497f, 25.648236545468112f, 27.049480737879435f, 27.512750132592018f, 29.703400302871156f, 32.99250004188777f, 35.295829006001384f, 36.09110061958704f, 36.229757152950306f, 35.6270983423867f, 32.95861483350798f, 28.369362585348345f, 24.215325029610522f, 20.44764438357939f, 14.041615241583967f, 2.4102601256836187f, -14.872190744947082f, -31.054273474029188f, -32.96945389856551f, -22.474569605379024f, -15.733261054358108f, -17.96200922839745f, -23.98780707979461f, -29.815321239689585f, -34.672930654067834f, -36.58181333720525f, -32.88658251612536f, -27.788282625312142f, -28.607490176651012f, -37.11743906011532f, -51.31474104000062f, -80.04734976282103f, -144.2767486514354f, -247.02136622210236f, -363.5151097387105f, -461.9391009112242f, -530.352034828062f, -575.1787772329582f, -597.7212420781298f, -597.2748485589892f, -576.128268699777f, -537.0077933327171f, -489.0661003187983f, -444.7410651320364f, -407.2459865151265f, -362.6321398393739f, -287.3474592268259f, -172.33726644838615f, -34.8198340584562f, 103.52658516729652f, 251.72823839357605f, 439.55764486181323f, 681.1917886753087f, 960.4818662874252f, 1242.3799616509834f, 1490.5889485278622f, 1686.0949355654566f, 1833.0815206571274f, 1937.4677485687082f, 1993.2031905484869f;
  velocities.row(1).setConstant(0);
  velocities.row(2) << -1.6754512029575988f, 0.5594371773002144f, 3.998451910938573f, 4.379526471594197f, 1.2454431427988208f, -3.3742529314846545f, -6.355339066345258f, -6.192139765134198f, -3.906004424566676f, -1.216615353510017f, 0.6317783995508084f, 1.3074396254398337f, 1.3540380980652706f, 1.4352308100860274f, 1.697781687050953f, 2.075222098254945f, 3.0434445728302357f, 4.601317873826512f, 4.42599863602666f, 1.1690793013317988f, -2.8025903105162335f, -4.459553500794235f, -3.9196180901300055f, -3.904260447955737f, -4.736289814001239f, -3.6261983562537865f, 0.9791184906091553f, 5.577454364245049f, 5.92943941973909f, 3.037449248777998f, -0.0537888904255227f, -2.0057482807554603f, -2.977366675561185f, -3.269224169421147f, -2.7277612273676746f, -1.0721998710809246f, 1.7826051729975283f, 5.166349114857415f, 7.130221470369452f, 5.755270429571883f, 1.4822404814283487f, -1.9815891416099285f, -1.4943491510740605f, 1.7131528089599086f, 3.2753928702937096f, 0.39538651987966955f, -5.582978050199133f, -11.75739789906418f, -15.326030625706379f, -14.089430662639524f, -7.910070264217452f, 4.14484245182023f, 25.919011491332128f, 55.88885760419718f, 82.23968298599111f, 90.51868465132398f, 82.13869649101908f, 70.7592657113157f, 60.19379737510449f, 48.39411228207252f, 37.24058562356361f, 30.331554181751805f, 28.8876804288692f, 32.80889067899143f, 45.566438456839585f, 71.21996620771763f, 109.16712847345627f, 154.8296342870756f, 205.10199130548628f, 259.86935465203436f, 316.24986141064664f, 369.3900056732338f, 416.3105413637785f, 456.67728961804966f, 489.1972076747233f, 508.7289297481959f, 516.0830918647708f, 517.338435420332f, 511.59757802674915f, 492.0992453742509f, 453.5593134083149f, 397.9929148939046f, 332.77708914203026f, 267.0601404761839f, 215.38154713394616f, 188.433385708341f, 169.01227855368282f, 99.0896063262312f, -66.8330094209048f, -306.5604520254367f, -545.2611609355525f, -711.7810235290298f, -777.027208299141f, -749.8285829111228f, -652.2828097398167f, -496.5781898278358f, -296.6433132968308f;

  DynamicMotionPrimitive<3>::Matrixf accelerations(3, 97);
  accelerations.row(0) << -372.28816715696775f, -422.2912742321245f, -195.0795669707204f, 211.35784798478787f, 237.45927612400038f, -3.4097208844325153f, -195.01476135499084f, -245.67746909010438f, -186.81740470590745f, 22.50043554120912f, 342.33579877678824f, 448.2161135303681f, 137.4559587328429f, -269.5678130727181f, -355.8082809327975f, -153.10817984984737f, -19.879838577768023f, -58.67206707252954f, -49.43309550419602f, 163.35526743913377f, 614.4836929792376f, 1218.722695060289f, 1550.979000084591f, 823.18574617017f, -1071.52719878946f, -2752.8745546694813f, -2811.4689406065995f, -1443.1845425111742f, -14.14392104393669f, 545.9178798441468f, 431.6228153101544f, 234.3862374413083f, 292.8699514435099f, 612.0065533640254f, 976.154843149475f, 1181.137282326213f, 1150.3533717815878f, 840.5490251558442f, 373.55926877252307f, 92.35441132482896f, 131.45582892015065f, 271.4268646660514f, 277.0081507157963f, 153.48208468977668f, 46.25999232550736f, -22.983290365997963f, -162.0285447948068f, -359.4953225448906f, -433.0788407537993f, -392.3841726109855f, -503.93141940692277f, -893.440528662127f, -1432.179175033781f, -1657.58904746241f, -896.406492655866f, 424.97598601725105f, 853.7553464887777f, 223.51934577572288f, -408.87003677395757f, -587.1266697182085f, -529.2631303144681f, -335.1626926806823f, 88.4826647765899f, 435.5674090937706f, 211.95504111414982f, -462.0984028453912f, -1124.7516782780644f, -2126.434829199441f, -4604.660190099105f, -8270.675581627953f, -10859.47022207998f, -10645.457886470522f, -8263.885074519276f, -5609.068079487756f, -3336.9794245360686f, -1094.4782993267715f, 1069.558494441773f, 2985.190585974224f, 4312.425162796139f, 4570.221116482317f, 4052.771992144491f, 4067.0776079543098f, 5938.89901521489f, 9425.820831516216f, 12508.377695255695f, 13664.284238907645f, 14193.502654166083f, 16644.5291250555f, 21272.493612085822f, 25802.78853790414f, 27797.171184776413f, 26257.640522189868f, 21978.40524996923f, 16964.58534845893f, 12451.17671885265f, 7931.260284338367f, 0.0f;
  accelerations.row(1).setConstant(0);
  accelerations.row(2) << 221.40015729656838f, 281.04379909952996f, 189.21937625942155f, -136.36398571159518f, -384.06570875063466f, -376.4873430697534f, -139.57757213404278f, 121.32218319090173f, 246.451209173908f, 224.7686819422679f, 125.02328400405801f, 35.77548039370701f, 6.3298437254656665f, 17.026551603963703f, 31.700503058834244f, 66.654329856357f, 125.12437019186267f, 68.48164985926212f, -170.0080788245045f, -358.0516020250218f, -278.8014378623175f, -55.32941338273823f, 27.50516990695365f, -40.45196389268727f, 13.773169028227443f, 283.09966368630927f, 455.8818637256432f, 245.20281237746403f, -125.8133374950969f, -296.36551442871445f, -249.80324211707782f, -144.8127314132618f, -62.58338513951531f, 12.363634349772019f, 108.82456804862782f, 223.41067216762218f, 309.0122394904038f, 264.8819287483289f, 29.170868859688593f, -279.75980599427896f, -383.22762361928596f, -147.43855189030623f, 183.01058259831902f, 236.25824965653442f, -65.27253581425482f, -438.77912036086036f, -601.9603497233869f, -482.5998004690505f, -115.511903242517f, 367.33261603636737f, 903.1929673517448f, 1675.6461056487174f, 2563.0213112859606f, 2789.7155039410522f, 1715.3091901847854f, -5.0021347976801795f, -978.7375736639611f, -1086.990330040629f, -1107.8066651868119f, -1136.934787693146f, -894.6874573056054f, -413.74203300822785f, 122.70919098476654f, 826.1440892359162f, 1902.604675721952f, 3150.3145522249383f, 4141.413465613058f, 4751.913766446346f, 5202.9020367689845f, 5505.4552481995315f, 5424.854676751001f, 4956.276670575689f, 4323.575746799289f, 3610.2741256823124f, 2578.258810184815f, 1331.7307122173077f, 426.4521501151495f, -222.17965739733594f, -1250.1654882638268f, -2874.792546520575f, -4661.341603232108f, -5982.670921601014f, -6485.4551814385f, -5814.919370493885f, -3894.596264201564f, -2296.795546499025f, -4425.439539487681f, -11682.056320236583f, -20092.9468155499f, -23697.84301894984f, -20071.673167934983f, -11480.000476888024f, -1884.598735748533f, 6178.928152938492f, 12544.178348985242f, 17615.788141568457f, 0.0f;

  trajectory.startPos = positions.col(0);
  trajectory.startVel = velocities.col(0);
  trajectory.endPos = positions.col(positions.cols() - 1);
  trajectory.endVel = velocities.col(velocities.cols() - 1);
  trajectory.executionTime = theFrameInfo.cycleTime * (positions.cols() - 1);
  kickState.imitatedDmp.initialize(trajectory.executionTime, trajectory.startPos,
                                   trajectory.startVel, DynamicMotionPrimitive<3>::Vectorf::Zero(),
                                   trajectory.endPos, trajectory.endVel,
                                   DynamicMotionPrimitive<3>::Vectorf::Zero(),
                                   trajectory.numWeights, trajectory.overlap,
                                   trajectory.finalPhaseValue, 4);
  DynamicMotionPrimitive<3>::Matrixf weights = kickState.imitatedDmp.imitate(positions, velocities, accelerations);
  kickState.imitatedDmp.setWeights(weights);

}

void DmpKickEngine::update(DmpKickEngineOutput& output)
{
  DECLARE_DEBUG_DRAWING("module:DmpKickEngine:zmpPreviews", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:DmpKickEngine:comTarget", "drawingOnField");

  DECLARE_VEC3_PLOT("module:DmpKickEngine:traj");
  DECLARE_VEC3_PLOT("module:DmpKickEngine:realTraj");
  DECLARE_VEC3_PLOT("module:DmpKickEngine:kickTarget");
  DECLARE_VEC3_PLOT("module:DmpKickEngine:kickTargetVelocity");

  this->output = &output;
  //set all joints we don't care about to whatever was requested
  for(int i = 0; i < Joints::numOfJoints; ++i)
  {
    output.angles[i] = theJointRequest.angles[i];
  }

  if(theMotionRequest.motion == MotionRequest::dmpKick)
  {
    for(int i = 0; i < Joints::numOfJoints; ++i)
    {
      ASSERT(output.angles[i] != JointAngles::off &&
             output.angles[i] != JointAngles::ignore);
    }
  }

  //If the motion request is unexpectedly changed (e.g. using SimRobot)
  //the engine would continue in a wrong state the next time. Therefore
  //the state is changed to Idle if the module is inactive.
  if(theMotionRequest.motion != MotionRequest::dmpKick)
  {
    state = Idle;
  }

  do
  {
    states[state]();
  } while(!nextFrame);

  //set head joints to ignore. This way the Head will be controlled by another model
  //this has to be done AFTER all calculations, otherwise the com calc will go wrong
  output.angles[Joints::headYaw] = JointAngles::ignore;
  output.angles[Joints::headPitch] = JointAngles::ignore;
}

void DmpKickEngine::goTo(State nextState)
{
  nextFrame = false;
  state = nextState;
}

void DmpKickEngine::goToNext(State nextState)
{
  nextFrame = true;
  state = nextState;
}

void DmpKickEngine::stateSetup()
{
  updateZmpParameters();
  goTo(Idle);
}

void DmpKickEngine::stateIdle()
{
  if(theMotionRequest.motion == MotionRequest::dmpKick && !kickState.movingBack)
  {
    goTo(InitKick);
  }
  else if(theMotionRequest.motion != MotionRequest::dmpKick && kickState.movingBack)
  {
    kickState.movingBack = false;
    output->isLeavingPossible = true;
    goToNext(Idle);
  }
  else
  {
    output->isLeavingPossible = true;
    goToNext(Idle);
  }
}


void DmpKickEngine::stateInitKick()
{
  kickState.leftSupport = theMotionRequest.dmpKickRequest.leftSupport;
  kickState.additionalKickSteps = additionalKickSteps;
  Limbs::Limb supportFoot = kickState.leftSupport ? Limbs::footLeft : Limbs::footRight;
  kickState.jointModel.init(theJointDataPrediction);
  kickState.robotModel.setJointData(kickState.jointModel.joints, theRobotDimensions, theMassCalibration);

  const Pose3f soleInTorso = Pose3f(kickState.robotModel.limbs[supportFoot]).translated(0, 0, -theRobotDimensions.footHeight);
  kickState.torsoInSupportSole = soleInTorso.inverse();
  kickState.comInSole = kickState.torsoInSupportSole * theJointDataPrediction.com;
  kickState.zmpInSole = kickState.leftSupport ? theZmp.zmpInLeftSole : theZmp.zmpInRightSole;

  targetIndex = 0;
  const float totalTime = (targets[0] - targets[1]).norm() / targetTime;
  totalSteps = int(totalTime / theFrameInfo.cycleTime);
  step = 1;
  from = 0;
  to = 1;

  initZmpControllers();

  targets[0] = kickState.comInSole.topRows(2);
  targets[targets.size() - 1] = kickState.comInSole.topRows(2);


  kickState.kickType = theMotionRequest.dmpKickRequest.type;
  //assume that we are starting unbalanced. If we are already balanced
  //loosing one motion frame does not hurt :D
  kickState.balanceReached = false;

  Limbs::Limb kickFoot = kickState.leftSupport ? Limbs::footRight : Limbs::footLeft;
  const Pose3f kickSoleInTorso = kickState.robotModel.limbs[kickFoot].translated(0, 0, -theRobotDimensions.footHeight);
  kickState.kickSoleInSupportSole = (kickState.torsoInSupportSole * kickSoleInTorso).translation;

  //the origin of the kick movement coordinate system is 10cm left/right of the support foot
  kickState.kickSoleZeroInSupportSole = Vector3f(0, kickState.leftSupport ? -100.f : 100.f, kickFootH);
  kickState.dmpStartInDmpZero = kickState.kickSoleInSupportSole - kickState.kickSoleZeroInSupportSole;

  for(int i = 0; i < Joints::numOfJoints; ++i)
  {
    output->stiffnessData.stiffnesses[i] = 100;
  }

  output->isLeavingPossible = false;

  switch(kickState.kickType)
  {
    case DmpKickRequest::Dynamic:
      goTo(InitDynKick);
      break;
    case DmpKickRequest::Imitated:
      goTo(InitImitatedKick);
      break;
    case DmpKickRequest::BalanceOnly:
      goTo(Balance);
      break;
    default:
      ASSERT(false);
      break;
  }
}

void DmpKickEngine::stateInitDynKick()
{
  kickState.dynKickMode = DmpKickState::windUp; //the first state of the dynKick is windUp.
  //FIXME start velocity and start acceleration should be calculated
  const Vector3f startVel = Vector3f::Zero();
  const Vector3f startAcc = Vector3f::Zero();
  const Vector3f endPos(xStartPos, 0, theFieldDimensions.ballRadius * 2.0f);
  kickState.dynDmp.initialize(0.3f, kickState.dmpStartInDmpZero, startVel, startAcc, endPos, Vector3f::Zero(),
                              Vector3f::Zero(), 30);
  //FIXME the kicktarget should be recalculated each frame. but there is a bug somewhere in the torso matrix
  kickState.kickTarget = getKickTarget();
  goToNext(DynKick);
}

void DmpKickEngine::stateInitImitatedKick()
{
  kickState.movingBack = false;
  //FIXME start velocity and start acceleration should be calculated
  const Vector3f startAcc = Vector3f::Zero();
  Vector3f endPos;
  if(theMotionRequest.dmpKickRequest.trackBall)
  {//FIXME this should happen every frame!
    endPos = getKickTarget();
  }
  else
  {
    endPos = trajectory.endPos;
  }
  //FIXMe use current startVel!
  kickState.imitatedDmp.reset(kickState.dmpStartInDmpZero, trajectory.startVel, startAcc);
  kickState.imitatedDmp.changeGoal(endPos, theMotionRequest.dmpKickRequest.kickSpeed);
  goToNext(ImitatedKick);
}

void DmpKickEngine::stateImitatedKick()
{
  if(kickState.balanceReached)
  {
    if(kickState.imitatedDmp.stepPossible())
    {
      const Vector3f nextPos = kickState.imitatedDmp.step(theFrameInfo.cycleTime);
      kickState.kickSoleInSupportSole = nextPos + kickState.kickSoleZeroInSupportSole;
    }
    else if(kickState.additionalKickSteps > 0)
    {
      //move target forward with the specified speed for a few more frames to give the 
      //robot time to catch up
      kickState.kickSoleInSupportSole.x() += theMotionRequest.dmpKickRequest.kickSpeed.x() * theFrameInfo.cycleTime;
      --kickState.additionalKickSteps;
    }
    else if(!kickState.movingBack)
    {
      //init moving back
      kickState.currentPreviewStep = 0;
      kickState.initialPreviewPos = theRobotModel.centerOfMass.topRows(2);
      const Vector3f start = kickState.kickSoleInSupportSole - kickState.kickSoleZeroInSupportSole;
      kickState.backDmp.initialize(moveBackTime, start, Vector3f::Zero(), Vector3f::Zero(),
                                   Vector3f(0, 0, moveBackEndHeight), Vector3f::Zero(), Vector3f::Zero(), 20);
      kickState.movingBack = true;
    }
    else if(kickState.backDmp.stepPossible())
    {
      const Vector3f nextPos = kickState.backDmp.step(theFrameInfo.cycleTime);
      kickState.kickSoleInSupportSole = nextPos + kickState.kickSoleZeroInSupportSole;
    }
  }
  PLOT_VEC3("module:DmpKickEngine:traj", kickState.kickSoleInSupportSole);
  PLOT_VEC3("module:DmpKickEngine:kickTarget", kickState.imitatedDmp.getEndPosition());
  PLOT_VEC3("module:DmpKickEngine:kickTargetVelocity", kickState.imitatedDmp.getEndVelocity());

  goTo(Balance);
}

void DmpKickEngine::stateDynKick()
{
  //not implemented on master branch
  ASSERT(false);
}

void DmpKickEngine::stateBalance()
{

  const float xRotDiff = theInertialSensorData.angle.x() - kickState.torsoInSupportSole.rotation.getXAngle();
  const float yRotDiff = theInertialSensorData.angle.y() - kickState.torsoInSupportSole.rotation.getYAngle();

  const float comInX = kickState.comInSole.x() + comP.y() * yRotDiff;
  const float comInY = kickState.comInSole.y() + comP.x() * xRotDiff;
  //input for the controller is the rotated com and rotated zmp
  const float comX = (float)kickState.zmpCtrlX.step(kickState.zmpInSole.x(), comInX,
                                                     kickState.xPreviews, balanceSensorFeedback,
                                                    Vector2f::Zero(), false) - comP.y() * yRotDiff;
  const float comY = (float)kickState.zmpCtrlY.step(kickState.zmpInSole.y(), comInY,
                                                     kickState.yPreviews, balanceSensorFeedback,
                                                     Vector2f::Zero(), false) - comP.x() * xRotDiff;

  Vector3f nextComInSole(comX, comY, theMotionSettings.comHeight);
  //rotate back to get

  if(kickState.kickType == DmpKickRequest::BalanceOnly)
  {//only used for experimentation
    kickState.kickSoleInSupportSole.z() = kickFootH;
  }

  ASSERT(nextComInSole.allFinite());
  setJoints(kickState.comInSole, nextComInSole, kickState.torsoInSupportSole, kickState.kickSoleInSupportSole);
  goToNext(IterateModel);

  COMPLEX_DRAWING("module:DmpKickEngine:comTarget")
  {
    Pose2f leftTransform(90_deg, Vector2f(-50, 0));
    Pose2f rightTransform(90_deg, Vector2f(50, 0));
    const Vector2f target = kickState.leftSupport ? (leftTransform * nextComInSole.topRows(2)) * 10 :
                                                    (rightTransform * nextComInSole.topRows(2)) * 10;
    RECTANGLE("module:DmpKickEngine:comTarget", target.x() - 10, target.y() - 10,
              target.x() + 10, target.y() + 10, 5, Drawings::solidPen, ColorRGBA::green);
  }
  COMPLEX_DRAWING("module:DmpKickEngine:zmpPreviews")
  {
    Pose2f leftTransform(90_deg, Vector2f(-50, 0));
    Pose2f rightTransform(90_deg, Vector2f(50, 0));
    for(size_t i = 0; i < kickState.xPreviews.size(); ++i)
    {
      const Vector2f p(kickState.xPreviews[i], kickState.yPreviews[i]);
      Vector2f spot;
      if(kickState.leftSupport)
      {
        spot = (leftTransform * p) * 10;
      }
      else
      {
        spot = (rightTransform * p) * 10;
      }
      CIRCLE("module:DmpKickEngine:zmpPreviews", spot.x(), spot.y(), 7, 9,
             Drawings::solidPen, ColorRGBA::red, Drawings::noBrush, ColorRGBA::red);
    }
  }
}

void DmpKickEngine::stateIterateModel()
{
  //iterate joint model and update corresponding data
  kickState.jointModel.update(*output, theFrameInfo.cycleTime);
  kickState.robotModel.setJointData(kickState.jointModel.joints, theRobotDimensions, theMassCalibration);

  Limbs::Limb supportFoot = kickState.leftSupport ? Limbs::footLeft : Limbs::footRight;
  const Pose3f soleInTorso = kickState.robotModel.limbs[supportFoot].translated(0, 0, -theRobotDimensions.footHeight);
  kickState.torsoInSupportSole = soleInTorso.inverse();

  kickState.comInSole = kickState.torsoInSupportSole * kickState.robotModel.centerOfMass;
  //FIXME sollte der zmp hier nicht dann auch irgendwie aus dem motormodell kommen?
  kickState.zmpInSole = kickState.leftSupport ? theZmp.zmpInLeftSole : theZmp.zmpInRightSole;
  Limbs::Limb kickFoot = kickState.leftSupport ? Limbs::footRight : Limbs::footLeft;
  const Pose3f realKickSoleInTorso = theRobotModel.limbs[kickFoot].translated(0, 0, -theRobotDimensions.footHeight);
  const Pose3f realTorsoInSupportSole = theRobotModel.limbs[supportFoot].translated(0, 0, -theRobotDimensions.footHeight).inverse();
  const Vector3f realKickSoleInSupportSole = (realTorsoInSupportSole * realKickSoleInTorso).translation;
  PLOT_VEC3("module:DmpKickEngine:realTraj", realKickSoleInSupportSole);

  updateZmpPreviews();
  checkBalanced();

  const Vector2f target(25, kickState.leftSupport ? -50 : 50);
  const float dist = (target - kickState.zmpInSole).norm();
  if(kickState.movingBack && dist < initiallyStableRadius)
  {
    goTo(Idle);
  }
  else
  {
    goToKick();
  }
}

void DmpKickEngine::checkBalanced()
{
  //check if we have reached a more or less balanced state, i.e.
  //the zmp is more or less inside the foot
  //FIXME zmp should be inside for several frames before assuming stability
  if(!kickState.balanceReached)
  {
    const Vector2f target(zmpTargetX, kickState.leftSupport ? zmpTargetY : -zmpTargetY);
    if((kickState.comInSole.topRows(2) - target).norm() < initiallyStableRadius)
      kickState.balanceReached = true;
  }
}
void DmpKickEngine::updateZmpPreviews()
{
  const Vector2f nextZmpPreview = getNextPreview();
  kickState.xPreviews.push_back(nextZmpPreview.x());
  kickState.yPreviews.push_back(nextZmpPreview.y());
  kickState.xPreviews.pop_front();
  kickState.yPreviews.pop_front();
  ASSERT(kickState.xPreviews.size() == numZmpPreviews);
  ASSERT(kickState.yPreviews.size() == numZmpPreviews);
}

void DmpKickEngine::goToKick()
{
  switch(kickState.kickType)
  {
    case DmpKickRequest::Dynamic:
      goTo(DynKick);
      break;
    case DmpKickRequest::Imitated:
      goTo(ImitatedKick);
      break;
    case DmpKickRequest::BalanceOnly:
      goTo(Balance);
      break;
    default:
      ASSERT(false); //added a new kick type? :D
  }
}

void DmpKickEngine::setJoints(Vector3f currentComInSole,
                              const Vector3f& nextComInSole,
                              Pose3f& torsoInSole,
                              const Vector3f& kickFootInSole) const
{
  const Pose3f swingInSole(kickFootInSole);
  RobotModel model;
  //do translation
  //move the desired com closer to the current com according to balanceRotation ;
  const Vector3f nextToCurrent = currentComInSole - nextComInSole;
  ASSERT(balanceRotation >= 0.0f && balanceRotation <= 1.0f);
  const Vector3f translationTarget = nextComInSole + (balanceRotation * nextToCurrent);
  for(int i = 0; i < 7; ++i)
  {
    const Vector3f comError = translationTarget - currentComInSole;
    torsoInSole.translate(comError);
    const Pose3f soleInTorso = torsoInSole.inverse();
    const Pose3f swingInTorso = soleInTorso * swingInSole;
    if(!InverseKinematic::calcLegJoints(kickState.leftSupport ? soleInTorso : swingInTorso,
                                        kickState.leftSupport ? swingInTorso : soleInTorso,
                                        Vector2f::Zero(), *output,
                                        theRobotDimensions))
    {
     // OUTPUT_WARNING("DmpKickEngine: InverseKinematic not reachable!");
    }
    model.setJointData(*output, theRobotDimensions, theMassCalibration);
    currentComInSole = torsoInSole * model.centerOfMass;
  }
  //do x rotation
  float z2 = std::sqrt(sqr(currentComInSole.y()) + sqr(currentComInSole.z()) - sqr(nextComInSole.y()));
  float angle = std::atan2(currentComInSole.y(), currentComInSole.z()) - std::atan2(nextComInSole.y(), z2);
  for(int i = 0; (i < 10) && (std::abs(angle) > 0.0005f); ++i)
  {
    torsoInSole.rotateX(angle);
    const Pose3f soleInTorso = torsoInSole.inverse();
    const Pose3f swingInTorso = soleInTorso * swingInSole;
    if(!InverseKinematic::calcLegJoints(kickState.leftSupport ? soleInTorso : swingInTorso,
                                        kickState.leftSupport ? swingInTorso : soleInTorso,
                                        Vector2f::Zero(), *output,
                                        theRobotDimensions))
    {
     // OUTPUT_WARNING("DmpKickEngine: InverseKinematic not reachable!");
    }
    model.setJointData(*output, theRobotDimensions, theMassCalibration);
    currentComInSole = torsoInSole * model.centerOfMass;
    z2 = std::sqrt(sqr(currentComInSole.y()) + sqr(currentComInSole.z()) - sqr(nextComInSole.y()));
    angle = std::atan2(currentComInSole.y(), currentComInSole.z()) - std::atan2(nextComInSole.y(), z2);
  }
  //y rotation
  z2 = std::sqrt(sqr(currentComInSole.x()) + sqr(currentComInSole.z()) - sqr(nextComInSole.x()));
  angle = std::atan2(currentComInSole.x(), currentComInSole.z()) - std::atan2(nextComInSole.x(), z2);
  for(int i = 0; (i < 10) && (std::abs(angle) > 0.0005f); ++i)
  {
    torsoInSole.rotateY(-angle);
    const Pose3f soleInTorso = torsoInSole.inverse();
    const Pose3f swingInTorso = soleInTorso * swingInSole;
    if(!InverseKinematic::calcLegJoints(kickState.leftSupport ? soleInTorso : swingInTorso,
                                        kickState.leftSupport ? swingInTorso : soleInTorso,
                                        Vector2f::Zero(), *output,
                                        theRobotDimensions))
    {
     // OUTPUT_WARNING("DmpKickEngine: InverseKinematic not reachable!");
    }
    model.setJointData(*output, theRobotDimensions, theMassCalibration);
    currentComInSole = torsoInSole * model.centerOfMass;
    z2 = std::sqrt(sqr(currentComInSole.x()) + sqr(currentComInSole.z()) - sqr(nextComInSole.x()));
    angle = std::atan2(currentComInSole.x(), currentComInSole.z()) - std::atan2(nextComInSole.x(), z2);
  }

  //the z translation is wrong after rotation, the following loop moves it back where it belongs
  Vector3f comError;
  for(int i = 0; i < 10; ++i)
  {
    comError = nextComInSole - currentComInSole;
    torsoInSole.translate(comError);
    const Pose3f soleInTorso = torsoInSole.inverse();
    const Pose3f swingInTorso = soleInTorso * swingInSole;
    if(!InverseKinematic::calcLegJoints(kickState.leftSupport ? soleInTorso : swingInTorso,
                                        kickState.leftSupport ? swingInTorso : soleInTorso,
                                        Vector2f::Zero(), *output,
                                        theRobotDimensions))
    {
      //OUTPUT_WARNING("DmpKickEngine: InverseKinematic not reachable!");
    }
    model.setJointData(*output, theRobotDimensions, theMassCalibration);
    currentComInSole = torsoInSole * model.centerOfMass;
  }
}


bool DmpKickEngine::parametersChanged()
{
  return !(theMotionSettings.comHeight == oldComHeight &&
           oldNumZmpPreviews == numZmpPreviews && oldR == R && oldQe == Qe &&
           oldQx == Qx && QlDiag.norm() == oldQlDiagNorm &&
           R0Diag.norm() == oldR0DiagNorm);
}

void DmpKickEngine::updateZmpParameters()
{
  kickState.zmpCtrlX.init(theFrameInfo.cycleTime, theMotionSettings.comHeight,
                          numZmpPreviews, R, Qx, Qe, QlDiag, R0Diag);
  kickState.zmpCtrlY.init(theFrameInfo.cycleTime, theMotionSettings.comHeight,
                          numZmpPreviews, R, Qx, Qe, QlDiag, R0Diag);
  //call zmp init here FIXME
  oldComHeight = theMotionSettings.comHeight;
  oldR = R;
  oldQe = Qe;
  oldQx = Qx;
  oldQlDiagNorm = QlDiag.norm();
  oldR0DiagNorm = R0Diag.norm();
  oldNumZmpPreviews = numZmpPreviews;
}

Vector3f DmpKickEngine::getKickTarget() const
{
  //calculate robot coordinate system origin in support foot coordinates
  //FIXME this is copy&pase code from TorsoMatrixProvider
  Pose3f fromRightFoot(theRobotModel.limbs[Limbs::footRight]);
  fromRightFoot.translate(0, 0, -theRobotDimensions.footHeight);
  fromRightFoot.translation *= -1.0f;
  Pose3f fromLeftFoot(theRobotModel.limbs[Limbs::footLeft]);
  fromRightFoot.translate(0, 0, -theRobotDimensions.footHeight);
  fromRightFoot.translation *= -1.0f;
  const Vector3f newFootSpan(fromRightFoot.translation - fromLeftFoot.translation);

  const Vector3f ballPos(theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), 0.0f);
  const Vector3f robotInSupport(newFootSpan.x() / (kickState.leftSupport ? 2.f : -2.f), newFootSpan.y() / (kickState.leftSupport ? 2.f : -2.f), 0);

  Vector3f ballOnGroundInSupportFoot = robotInSupport + ballPos;
  ballOnGroundInSupportFoot.z() = theFieldDimensions.ballRadius;

  const Vector3f ballOnGroundInKickZero = ballOnGroundInSupportFoot - kickState.kickSoleZeroInSupportSole;
  const Vector3f kickTarget(ballOnGroundInKickZero.x() - footLength -
                            theFieldDimensions.ballRadius,
                            ballOnGroundInKickZero.y(), theFieldDimensions.ballRadius);

  return kickTarget;
}

float DmpKickEngine::calcTimeForSpeed(const float endSpeed, const float startPos, const float endPos) const
{
  /**A dmp without forces is shaped by a 5'th degree polynomial. The polynomial
     will contain a bump when the execution time is too short/long for the
     requested kick speed. This method uses a second degree polynomial to
     calculate a good value for the time to avoid the bump.*/

  // (1) use f(t) = at^2 + y0 as polynomial
  // (2) derive f(t): f(t)'=2at
  // (3) enter the parameters into both: endPos = at^^2 + startPos
  //                                     endSpeed = 2at
  // (4) solve for t
  //
  const float t = 2 * (endPos - startPos) / endSpeed;
  return t;
}

void DmpKickEngine::initZmpControllers()
{
  if(parametersChanged())
  {
    updateZmpParameters();
  }
  //FIXME initial com velocity might not be 0 
  kickState.zmpCtrlX.reset(kickState.comInSole.x(), 0.0f, kickState.zmpInSole.x());
  kickState.zmpCtrlY.reset(kickState.comInSole.y(), 0.0f, kickState.zmpInSole.y());

  kickState.xPreviews.clear();
  kickState.yPreviews.clear();

  kickState.initialPreviewPos = kickState.zmpInSole;
  kickState.previewSteps = int(balanceTime / theFrameInfo.cycleTime);
  kickState.currentPreviewStep = 0;

  for(size_t i = 0; i < numZmpPreviews; ++i)
  {
    const Vector2f nextZmp = getNextPreview();
    kickState.xPreviews.push_back(nextZmp.x());
    kickState.yPreviews.push_back(nextZmp.y());
  }
}

Vector2f DmpKickEngine::getNextPreview()
{
  if(kickState.movingBack)
  {
    const Vector2f target(25, kickState.leftSupport ? -50 : 50);
    if(kickState.currentPreviewStep < kickState.previewSteps)
    {
      const Vector2f dist = target - kickState.initialPreviewPos;
      const Vector2f next = kickState.initialPreviewPos + dist *
                            ((float)kickState.currentPreviewStep / (float)kickState.previewSteps);
      ++kickState.currentPreviewStep;
      return next;
    }
    else
    {
      return target;
    }
  }
  else
  {
    const Vector2f target(zmpTargetX, kickState.leftSupport ? zmpTargetY : -zmpTargetY);
    if(kickState.currentPreviewStep < kickState.previewSteps)
    {
      const Vector2f dist = target - kickState.initialPreviewPos;
      const Vector2f next = kickState.initialPreviewPos + dist *
                            ((float)kickState.currentPreviewStep / (float)kickState.previewSteps);
      ++kickState.currentPreviewStep;
      return next;
    }
    else
    {
      return target;
    }
  }
}
