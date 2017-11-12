/*************************************************************************
 *                                                                       *
 * ODER's Utilities Library. Copyright (C) 2008 Oleh Derevenko.          *
 * All rights reserved.  e-mail: odar@eleks.com (change all "a" to "e")  *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 3 of the License, or (at    *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE-LESSER.TXT. Since LGPL is the extension of GPL     *
 *       the text of GNU General Public License is also provided for     *
 *       your information in file LICENSE.TXT.                           *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *   (3) The zlib/libpng license that is included with this library in   *
 *       the file LICENSE-ZLIB.TXT                                       *
 *                                                                       *
 * This library is distributed WITHOUT ANY WARRANTY, including implied   *
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.      *
 * See the files LICENSE.TXT and LICENSE-LESSER.TXT or LICENSE-BSD.TXT   *
 * or LICENSE-ZLIB.TXT for more details.                                 *
 *                                                                       *
 *************************************************************************/

#include <ou/features.h>
#include <ou/platform.h>

#if _OU_COMPILER == _OU_COMPILER_MSVC

#pragma warning(disable:4786)


#endif

#if _OU_FEATURE_SET >= _OU_FEATURE_SET_TLS
#include <ou/threadlocalstorage.h>
#endif
#if _OU_FEATURE_SET >= _OU_FEATURE_SET_ATOMICS
#include <ou/atomic.h>
#endif
#if _OU_FEATURE_SET >= _OU_FEATURE_SET_ATOMICS
#include <ou/atomicflags.h>
#endif
#include <ou/simpleflags.h>
#include <ou/flagsdefines.h>
#include <ou/enumarrays.h>
#include <ou/templates.h>
#include <ou/typewrapper.h>
#include <ou/customization.h>
#include <ou/inttypes.h>
#include <ou/macros.h>
#include <ou/malloc.h>

#include <ou/namespace.h>
using namespace _OU_NAMESPACE;

#include <stdio.h>
#include <string.h>


//////////////////////////////////////////////////////////////////////////

typedef bool (*CFeatureTestProcedure)();


bool TestSubsystem(unsigned int &nOutSuccessCount, unsigned int &nOutTestCount,
	const unsigned int uiFeatureMax, const char *const *aszFeatureNames, CFeatureTestProcedure const *afnFeatureTests)
{
	unsigned int nSuccessCount = 0;
	
	for (unsigned int uiSubsystemFeature = 0; uiSubsystemFeature != uiFeatureMax; ++uiSubsystemFeature)
	{
		const char *szFeatureName = aszFeatureNames[uiSubsystemFeature];
		printf("Testing %34s: ", szFeatureName);
		
		CFeatureTestProcedure fnTestProcedure = afnFeatureTests[uiSubsystemFeature];
		bool bTestResult = fnTestProcedure();
		printf("%s\n", bTestResult ? "success" : "*** failure ***");
		
		if (bTestResult)
		{
			nSuccessCount += 1;
		}
	}
	
	nOutSuccessCount = nSuccessCount;
	nOutTestCount = uiFeatureMax;
	return nSuccessCount == uiFeatureMax;
}



//////////////////////////////////////////////////////////////////////////

#if _OU_FEATURE_SET >= _OU_FEATURE_SET_TLS

bool g_bTestTLSAPIInitialized = false;
HTLSKEY g_htkTestTLSKey;

enum ETESTTLSVALUES
{
	TTV_FIRSTVALUE,
	TTV_SECONDVALUE,

	TTV__MAX,
};

unsigned int g_uiTestTLSDestructorCallCount = 0;
unsigned int g_uiTestTLSDestructorSuccessCount = 0;

void _OU_CONVENTION_CALLBACK TestTlsSecondValueDestructor(void *pv_Value)
{
	g_uiTestTLSDestructorCallCount += 1;

	if (pv_Value == (void *)(&TestTlsSecondValueDestructor))
	{
		g_uiTestTLSDestructorSuccessCount += 1;
	}
}


bool TestTls_Initialization()
{
	bool bResult = false;
	
	do
	{
		if (!CTLSInitialization::InitializeTLSAPI(g_htkTestTLSKey, 1, 0))
		{
			break;
		}

		CTLSInitialization::FinalizeTLSAPI();

		if (!CTLSInitialization::InitializeTLSAPI(g_htkTestTLSKey, TTV__MAX, CTLSInitialization::SIF_MANUAL_CLEANUP_ON_THREAD_EXIT))
		{
			break;
		}

		g_bTestTLSAPIInitialized = true;
	
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestTls_GetSetValue()
{
	bool bResult = false;
	
	do
	{
		tlsvaluetype vtFirstValue = CThreadLocalStorage::GetStorageValue(g_htkTestTLSKey, TTV_FIRSTVALUE);

		if (vtFirstValue != 0)
		{
			break;
		}
	
		tlsvaluetype vtSecondValue = CThreadLocalStorage::GetStorageValue(g_htkTestTLSKey, TTV_SECONDVALUE);
		
		if (vtSecondValue != 0)
		{
			break;
		}

		if (!CThreadLocalStorage::SetStorageValue(g_htkTestTLSKey, TTV_FIRSTVALUE, (tlsvaluetype)&TestTls_GetSetValue))
		{
			break;
		}
		
		if (!CThreadLocalStorage::SetStorageValue(g_htkTestTLSKey, TTV_SECONDVALUE, (tlsvaluetype)&TestTlsSecondValueDestructor, &TestTlsSecondValueDestructor))
		{
			break;
		}

		vtFirstValue = CThreadLocalStorage::GetStorageValue(g_htkTestTLSKey, TTV_FIRSTVALUE);

		if ((void *)vtFirstValue != &TestTls_GetSetValue)
		{
			break;
		}

		vtSecondValue = CThreadLocalStorage::GetStorageValue(g_htkTestTLSKey, TTV_SECONDVALUE);

		if ((void *)vtSecondValue != &TestTlsSecondValueDestructor)
		{
			break;
		}
		
		if (!CThreadLocalStorage::SetStorageValue(g_htkTestTLSKey, TTV_SECONDVALUE, 0, &TestTlsSecondValueDestructor))
		{
			break;
		}

		vtSecondValue = CThreadLocalStorage::GetStorageValue(g_htkTestTLSKey, TTV_SECONDVALUE);
		
		if (vtSecondValue != 0)
		{
			break;
		}
		
		if (!CThreadLocalStorage::SetStorageValue(g_htkTestTLSKey, TTV_SECONDVALUE, (tlsvaluetype)&TestTlsSecondValueDestructor, &TestTlsSecondValueDestructor))
		{
			break;
		}
		
		if (g_uiTestTLSDestructorCallCount != 0)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestTls_UnsafeGetSetValue()
{
	bool bResult = false;
	
	do
	{
		tlsvaluetype vtFirstValue = CThreadLocalStorage::UnsafeGetStorageValue(g_htkTestTLSKey, TTV_FIRSTVALUE);
		
		if ((void *)vtFirstValue != &TestTls_GetSetValue)
		{
			break;
		}
		
		tlsvaluetype vtSecondValue = CThreadLocalStorage::UnsafeGetStorageValue(g_htkTestTLSKey, TTV_SECONDVALUE);
		
		if ((void *)vtSecondValue != &TestTlsSecondValueDestructor)
		{
			break;
		}

		CThreadLocalStorage::UnsafeSetStorageValue(g_htkTestTLSKey, TTV_FIRSTVALUE, (tlsvaluetype)(size_t)(-1));
		CThreadLocalStorage::UnsafeSetStorageValue(g_htkTestTLSKey, TTV_SECONDVALUE, (tlsvaluetype)(size_t)(-1));

		vtFirstValue = CThreadLocalStorage::UnsafeGetStorageValue(g_htkTestTLSKey, TTV_FIRSTVALUE);
		
		if ((size_t)vtFirstValue != (size_t)(-1))
		{
			break;
		}
		
		vtSecondValue = CThreadLocalStorage::UnsafeGetStorageValue(g_htkTestTLSKey, TTV_SECONDVALUE);
		
		if ((size_t)vtSecondValue != (size_t)(-1))
		{
			break;
		}
		
		// Safe function used by intent !!!
		vtFirstValue = CThreadLocalStorage::GetStorageValue(g_htkTestTLSKey, TTV_FIRSTVALUE);
		
		if ((size_t)vtFirstValue != (size_t)(-1))
		{
			break;
		}
		
		// Safe function used by intent !!!
		vtSecondValue = CThreadLocalStorage::GetStorageValue(g_htkTestTLSKey, TTV_SECONDVALUE);
		
		if ((size_t)vtSecondValue != (size_t)(-1))
		{
			break;
		}
	
		CThreadLocalStorage::UnsafeSetStorageValue(g_htkTestTLSKey, TTV_SECONDVALUE, (tlsvaluetype)(&TestTlsSecondValueDestructor));

		if (g_uiTestTLSDestructorCallCount != 0)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestTls_CleanupDestructor()
{
	bool bResult = false;
	
	do
	{
		CTLSInitialization::CleanupOnThreadExit();

		if (g_uiTestTLSDestructorCallCount != 1 || g_uiTestTLSDestructorSuccessCount != 1)
		{
			break;
		}
		
		tlsvaluetype vtFirstValue = CThreadLocalStorage::GetStorageValue(g_htkTestTLSKey, TTV_FIRSTVALUE);
		
		if (vtFirstValue != 0)
		{
			break;
		}
		
		// Safe function used by intent !!!
		tlsvaluetype vtSecondValue = CThreadLocalStorage::GetStorageValue(g_htkTestTLSKey, TTV_SECONDVALUE);
		
		if (vtSecondValue != 0)
		{
			break;
		}

		g_uiTestTLSDestructorCallCount = 0;
		g_uiTestTLSDestructorSuccessCount = 0;

		CTLSInitialization::CleanupOnThreadExit();
		
		if (g_uiTestTLSDestructorCallCount != 0)
		{
			break;
		}
		
		if (!CThreadLocalStorage::SetStorageValue(g_htkTestTLSKey, TTV_SECONDVALUE, 0, &TestTlsSecondValueDestructor))
		{
			break;
		}
	
		CTLSInitialization::CleanupOnThreadExit();
		
		if (g_uiTestTLSDestructorCallCount != 0)
		{
			break;
		}
		
		if (!CThreadLocalStorage::SetStorageValue(g_htkTestTLSKey, TTV_SECONDVALUE, (tlsvaluetype)(size_t)(-1), &TestTlsSecondValueDestructor))
		{
			break;
		}
		
		if (g_uiTestTLSDestructorCallCount != 0)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestTls_Finalization()
{
	OU_ASSERT(g_bTestTLSAPIInitialized);

	bool bResult = false;
	
	do
	{
		CTLSInitialization::FinalizeTLSAPI();
		
		g_bTestTLSAPIInitialized = false;

		if (g_uiTestTLSDestructorCallCount != 1 || g_uiTestTLSDestructorSuccessCount != 0)
		{
			break;
		}
	
		bResult = true;
	}
	while (false);
	
	return bResult;
}


enum EOUTLSFEATURE
{
	OHF__MIN,
	
	OHF_INITIALIZATION = OHF__MIN,
	OHF_GETSETVALUE,
	OHF_UNSAFEGETSETVALUE,
	OHF_CLEANUPDESTRUCTOR,
	OHF_FINALIZATION,
	
	OHF__MAX,
};

template<>
CFeatureTestProcedure const CEnumUnsortedElementArray<EOUTLSFEATURE, OHF__MAX, CFeatureTestProcedure>::m_aetElementArray[] =
{
	&TestTls_Initialization, // OHF_INITIALIZATION
	&TestTls_GetSetValue, // OHF_GETSETVALUE,
	&TestTls_UnsafeGetSetValue, // OHF_UNSAFEGETSETVALUE,
	&TestTls_CleanupDestructor, // OHF_CLEANUPDESTRUCTOR,
	&TestTls_Finalization, // OHF_FINALIZATION,
};
static const CEnumUnsortedElementArray<EOUTLSFEATURE, OHF__MAX, CFeatureTestProcedure> g_afnTlsFeatureTestProcedures;

template<>
const char *const CEnumUnsortedElementArray<EOUTLSFEATURE, OHF__MAX, const char *>::m_aetElementArray[] =
{
	"API Initialization", // OHF_INITIALIZATION
	"Get.../SetStorageValue", // OHF_GETSETVALUE,
	"UnsafeGet.../UnsafeSetStorageValue", // OHF_UNSAFEGETSETVALUE,
	"Storage Cleanup/Value Destructors", // OHF_CLEANUPDESTRUCTOR,
	"API Finalization", // OHF_FINALIZATION,
};
static const CEnumUnsortedElementArray<EOUTLSFEATURE, OHF__MAX, const char *> g_aszTlsFeatureTestNames;


bool TestTLS(unsigned int &nOutSuccessCount, unsigned int &nOutTestCount)
{
	bool bResult = TestSubsystem(nOutSuccessCount, nOutTestCount, OHF__MAX, g_aszTlsFeatureTestNames.GetStoragePointer(), g_afnTlsFeatureTestProcedures.GetStoragePointer());

	if (g_bTestTLSAPIInitialized)
	{
		CTLSInitialization::FinalizeTLSAPI();
	}
	
	return bResult;
}


#endif // #if _OU_FEATURE_SET >= _OU_FEATURE_SET_TLS


//////////////////////////////////////////////////////////////////////////

#if _OU_FEATURE_SET >= _OU_FEATURE_SET_ATOMICS

bool TestAtomic_Increment()
{
	bool bResult = false;
	
	do
	{
		volatile atomicord32 aoStorage = (atomicord32)(-1);

		// Putting function inside of conditional operator causes
		// incorrect code generation by GCC 4.0.1 on MacOS X Leopard 64 bit.
		atomicord32 aoIncrementFirstResult = AtomicIncrement(&aoStorage);
		if (aoIncrementFirstResult != 0 || aoStorage != (atomicord32)0)
		{
			break;
		}
		
		if (AtomicIncrement(&aoStorage) != (atomicord32)1 || aoStorage != (atomicord32)1)
		{
			break;
		}
	
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomic_Decrement()
{
	bool bResult = false;
	
	do
	{
		volatile atomicord32 aoStorage = (atomicord32)1;
		
		// Putting function inside of conditional operator causes
		// incorrect code generation by GCC 4.0.1 on MacOS X Leopard 64 bit.
		atomicord32 aoDecrementFirstResult = AtomicDecrement(&aoStorage);
		if (aoDecrementFirstResult != (atomicord32)0 || aoStorage != (atomicord32)0)
		{
			break;
		}
		
		if (AtomicDecrement(&aoStorage) != (atomicord32)(-1) || aoStorage != (atomicord32)(-1))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomic_IncrementNoResult()
{
	bool bResult = false;
	
	do
	{
		volatile atomicord32 aoStorage = (atomicord32)(-1);
		
		AtomicIncrementNoResult(&aoStorage);

		if (aoStorage != (atomicord32)0)
		{
			break;
		}
		
		AtomicIncrementNoResult(&aoStorage);

		if (aoStorage != (atomicord32)1)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomic_DecrementNoResult()
{
	bool bResult = false;
	
	do
	{
		volatile atomicord32 aoStorage = (atomicord32)1;
		
		AtomicDecrementNoResult(&aoStorage);

		if (aoStorage != (atomicord32)0)
		{
			break;
		}
		
		AtomicDecrementNoResult(&aoStorage);

		if (aoStorage != (atomicord32)(-1))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomic_Exchange()
{
	bool bResult = false;
	
	do
	{
		volatile atomicord32 aoStorage = 0;

		if (AtomicExchange(&aoStorage, (atomicord32)1) != 0 || aoStorage != (atomicord32)1)
		{
			break;
		}
		
		if (AtomicExchange(&aoStorage, (atomicord32)(-1)) != (atomicord32)1 || aoStorage != (atomicord32)(-1))
		{
			break;
		}
		
		if (AtomicExchange(&aoStorage, 0) != (atomicord32)(-1) || aoStorage != 0)
		{
			break;
		}
		
		if (AtomicExchange(&aoStorage, 0) != 0 || aoStorage != 0)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;

}

bool TestAtomic_ExchangeAdd()
{
	bool bResult = false;
	
	do
	{
		volatile atomicord32 aoStorage = 0;

		if (AtomicExchangeAdd(&aoStorage, (atomicord32)1) != 0 || aoStorage != (atomicord32)1)
		{
			break;
		}
		
		if (AtomicExchangeAdd(&aoStorage, (atomicord32)(-2)) != 1 || aoStorage != (atomicord32)(-1))
		{
			break;
		}
		
		if (AtomicExchangeAdd(&aoStorage, (atomicord32)1) != (atomicord32)(-1) || aoStorage != 0)
		{
			break;
		}
	
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomic_ExchangeAddNoResult()
{
	bool bResult = false;
	
	do
	{
		volatile atomicord32 aoStorage = 0;
		
		AtomicExchangeAddNoResult(&aoStorage, (atomicord32)1);
		
		if (aoStorage != (atomicord32)1)
		{
			break;
		}
		
		AtomicExchangeAddNoResult(&aoStorage, (atomicord32)(-2));

		if (aoStorage != (atomicord32)(-1))
		{
			break;
		}
		
		AtomicExchangeAddNoResult(&aoStorage, (atomicord32)1);
		
		if (aoStorage != 0)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomic_CompareExchange()
{
	bool bResult = false;
	
	do
	{
		volatile atomicord32 aoStorage = 0;

		if (AtomicCompareExchange(&aoStorage, 1, 1) || aoStorage != 0)
		{
			break;
		}
		
		if (!AtomicCompareExchange(&aoStorage, 0, 1) || aoStorage != 1)
		{
			break;
		}
		
		if (!AtomicCompareExchange(&aoStorage, 1, 1) || aoStorage != 1)
		{
			break;
		}
		
		if (!AtomicCompareExchange(&aoStorage, 1, (atomicord32)(-1)) || aoStorage != (atomicord32)(-1))
		{
			break;
		}
		
		if (AtomicCompareExchange(&aoStorage, 1, (atomicord32)(-1)) || aoStorage != (atomicord32)(-1))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

const atomicord32 g_aoBitmask = (atomicord32)(OU_INT32_MIN + 1);

bool TestAtomic_And()
{
	bool bResult = false;
	
	do
	{
		volatile atomicord32 aoStorage = (atomicord32)OU_UINT32_MAX;

		if (AtomicAnd(&aoStorage, g_aoBitmask) != (atomicord32)OU_UINT32_MAX || aoStorage != g_aoBitmask)
		{
			break;
		}

		if (AtomicAnd(&aoStorage, 0) != g_aoBitmask || aoStorage != 0)
		{
			break;
		}
	
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomic_Or()
{
	bool bResult = false;
	
	do
	{
		volatile atomicord32 aoStorage = 0;
		
		if (AtomicOr(&aoStorage, g_aoBitmask) != 0 || aoStorage != g_aoBitmask)
		{
			break;
		}
		
		if (AtomicOr(&aoStorage, (atomicord32)OU_UINT32_MAX) != g_aoBitmask || aoStorage != (atomicord32)OU_UINT32_MAX)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomic_Xor()
{
	bool bResult = false;
	
	do
	{
		volatile atomicord32 aoStorage = 0;
		
		if (AtomicXor(&aoStorage, g_aoBitmask) != 0 || aoStorage != g_aoBitmask)
		{
			break;
		}
		
		if (AtomicXor(&aoStorage, (atomicord32)OU_UINT32_MAX) != g_aoBitmask || aoStorage != (atomicord32)(OU_UINT32_MAX ^ g_aoBitmask))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomic_AndNoResult()
{
	bool bResult = false;
	
	do
	{
		volatile atomicord32 aoStorage = (atomicord32)OU_UINT32_MAX;
		
		AtomicAndNoResult(&aoStorage, g_aoBitmask);

		if (aoStorage != g_aoBitmask)
		{
			break;
		}
		
		AtomicAndNoResult(&aoStorage, 0);

		if (aoStorage != 0)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomic_OrNoResult()
{
	bool bResult = false;
	
	do
	{
		volatile atomicord32 aoStorage = 0;
		
		AtomicOrNoResult(&aoStorage, g_aoBitmask);

		if (aoStorage != g_aoBitmask)
		{
			break;
		}
		
		AtomicOrNoResult(&aoStorage, (atomicord32)OU_UINT32_MAX);

		if (aoStorage != (atomicord32)OU_UINT32_MAX)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomic_XorNoResult()
{
	bool bResult = false;
	
	do
	{
		volatile atomicord32 aoStorage = 0;
		
		AtomicXorNoResult(&aoStorage, g_aoBitmask);

		if (aoStorage != g_aoBitmask)
		{
			break;
		}
		
		AtomicXorNoResult(&aoStorage, (atomicord32)OU_UINT32_MAX);

		if (aoStorage != (atomicord32)(OU_UINT32_MAX ^ g_aoBitmask))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomic_ExchangePointer()
{
	bool bResult = false;
	
	do
	{
		volatile atomicptr apStorage = NULL;

		if (AtomicExchangePointer(&apStorage, (atomicptr)(&TestAtomic_ExchangePointer)) != NULL || apStorage != (atomicptr)(&TestAtomic_ExchangePointer))
		{
			break;
		}
	
		if (AtomicExchangePointer(&apStorage, (atomicptr)(&apStorage)) != (atomicptr)(&TestAtomic_ExchangePointer) || apStorage != (atomicptr)(&apStorage))
		{
			break;
		}
		
		if (AtomicExchangePointer(&apStorage, NULL) != (atomicptr)(&apStorage) || apStorage != NULL)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomic_CompareExchangePointer()
{
	bool bResult = false;
	
	do
	{
		volatile atomicptr apStorage = NULL;
		
		if (AtomicCompareExchangePointer(&apStorage, (atomicptr)(&TestAtomic_CompareExchangePointer), (atomicptr)(&TestAtomic_CompareExchangePointer)) || apStorage != NULL)
		{
			break;
		}
		
		if (!AtomicCompareExchangePointer(&apStorage, NULL, (atomicptr)(&TestAtomic_CompareExchangePointer)) || apStorage != (atomicptr)(&TestAtomic_CompareExchangePointer))
		{
			break;
		}
		
		if (!AtomicCompareExchangePointer(&apStorage, (atomicptr)(&TestAtomic_CompareExchangePointer), (atomicptr)(&apStorage)) || apStorage != (atomicptr)(&apStorage))
		{
			break;
		}
		
		if (!AtomicCompareExchangePointer(&apStorage, (atomicptr)(&apStorage), (atomicptr)(&apStorage)) || apStorage != (atomicptr)(&apStorage))
		{
			break;
		}
		
		if (AtomicCompareExchangePointer(&apStorage, NULL, NULL) || apStorage != (atomicptr)(&apStorage))
		{
			break;
		}
		
		if (!AtomicCompareExchangePointer(&apStorage, (atomicptr)(&apStorage), NULL) || apStorage != NULL)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}


enum EOUATOMICFEATURE
{
	OOF__MIN,
		
	OOF_INCREMENT = OOF__MIN,
	OOF_DECREMENT,
	OOF_INCREMENTNORESULT,
	OOF_DECREMENTNORESULT,
	OOF_EXCHANGE,
	OOF_EXCHANGEADD,
	OOF_EXCHANGEADDNORESULT,
	OOF_COMPAREEXCHANGE,
	OOF_AND,
	OOF_OR,
	OOF_XOR,
	OOF_ANDNORESULT,
	OOF_ORNORESULT,
	OOF_XORNORESULT,
	OOF_EXCHANGEPOINTER,
	OOF_COMPAREEXCHANGEPOINTER,
	
	OOF__MAX,
};

template<>
CFeatureTestProcedure const CEnumUnsortedElementArray<EOUATOMICFEATURE, OOF__MAX, CFeatureTestProcedure>::m_aetElementArray[] =
{
	&TestAtomic_Increment, // OOF_INCREMENT,
	&TestAtomic_Decrement, // OOF_DECREMENT,
	&TestAtomic_IncrementNoResult, // OOF_INCREMENTNORESULT,
	&TestAtomic_DecrementNoResult, // OOF_DECREMENTNORESULT,
	&TestAtomic_Exchange, // OOF_EXCHANGE,
	&TestAtomic_ExchangeAdd, // OOF_EXCHANGEADD,
	&TestAtomic_ExchangeAddNoResult, // OOF_EXCHANGEADDNORESULT,
	&TestAtomic_CompareExchange, // OOF_COMPAREEXCHANGE,
	&TestAtomic_And, // OOF_AND,
	&TestAtomic_Or, // OOF_OR,
	&TestAtomic_Xor, // OOF_XOR,
	&TestAtomic_AndNoResult, // OOF_ANDNORESULT,
	&TestAtomic_OrNoResult, // OOF_ORNORESULT,
	&TestAtomic_XorNoResult, // OOF_XORNORESULT,
	&TestAtomic_ExchangePointer, // OOF_EXCHANGEPOINTER,
	&TestAtomic_CompareExchangePointer, // OOF_COMPAREEXCHANGEPOINTER,
};
static const CEnumUnsortedElementArray<EOUATOMICFEATURE, OOF__MAX, CFeatureTestProcedure> g_afnAtomicFeatureTestProcedures;

template<>
const char *const CEnumUnsortedElementArray<EOUATOMICFEATURE, OOF__MAX, const char *>::m_aetElementArray[] =
{
	"AtomicIncrement", // OOF_INCREMENT,
	"AtomicDecrement", // OOF_DECREMENT,
	"AtomicIncrementNoResult", // OOF_INCREMENTNORESULT,
	"AtomicDecrementNoResult", // OOF_DECREMENTNORESULT,
	"AtomicExchange", // OOF_EXCHANGE,
	"AtomicExchangeAdd", // OOF_EXCHANGEADD,
	"AtomicExchangeAddNoResult", // OOF_EXCHANGEADDNORESULT,
	"AtomicCompareExchange", // OOF_COMPAREEXCHANGE,
	"AtomicAnd", // OOF_AND,
	"AtomicOr", // OOF_OR,
	"AtomicXor", // OOF_XOR,
	"AtomicAndNoResult", // OOF_ANDNORESULT,
	"AtomicOrNoResult", // OOF_ORNORESULT,
	"AtomicXorNoResult", // OOF_XORNORESULT,
	"AtomicExchangePointer", // OOF_EXCHANGEPOINTER,
	"AtomicCompareExchangePointer", // OOF_COMPAREEXCHANGEPOINTER,
};
static const CEnumUnsortedElementArray<EOUATOMICFEATURE, OOF__MAX, const char *> g_aszAtomicFeatureTestNames;


bool TestAtomic(unsigned int &nOutSuccessCount, unsigned int &nOutTestCount)
{
	bool bResult = false;
	nOutSuccessCount = 0;
	nOutTestCount = OOF__MAX;

	bool bAPIInitialized = false;
	
	do
	{
		if (!InitializeAtomicAPI())
		{
			break;
		}
		
		bAPIInitialized = true;

		if (!TestSubsystem(nOutSuccessCount, nOutTestCount, OOF__MAX, g_aszAtomicFeatureTestNames.GetStoragePointer(), g_afnAtomicFeatureTestProcedures.GetStoragePointer()))
		{
			break;
		}

		bResult = true;
	}
	while (false);
	
	if (bAPIInitialized)
	{
		FinalizeAtomicAPI();
	}

	return bResult;
}


#endif // #if _OU_FEATURE_SET >= _OU_FEATURE_SET_ATOMICS


//////////////////////////////////////////////////////////////////////////

#if _OU_FEATURE_SET >= _OU_FEATURE_SET_ATOMICS

const atomicord32 g_aoTestValue32 = (atomicord32)0xA5A5A5A5;
const atomicord32 g_aoTestMask32 = (atomicord32)0xC6C6C6C6;
const atomicord32 g_aoTestBit32 = (atomicord32)OU_INT32_MIN;
const atomicord32 g_aoTestAnotherBit32 = (atomicord32)((uint32ou)OU_INT32_MIN >> 1);


bool TestAtomicFlags_Constructors()
{
	bool bResult = false;
	
	do
	{
		if (sizeof(CAtomicFlags::value_type) != sizeof(atomicord32))
		{
			break;
		}

		CAtomicFlags afEmptyFlags;

		if (afEmptyFlags.QueryFlagsAllValues())
		{
			break;
		}

		CAtomicFlags afFullFlags(OU_UINT32_MAX);
		
		if (afFullFlags.QueryFlagsAllValues() != (atomicord32)OU_UINT32_MAX)
		{
			break;
		}

		CAtomicFlags afCopyOfFullFlags(afFullFlags);

		if (afCopyOfFullFlags.QueryFlagsAllValues() != (atomicord32)OU_UINT32_MAX)
		{
			break;
		}
	
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomicFlags_AssignFlagsAllValues()
{
	bool bResult = false;
	
	do
	{
		CAtomicFlags afTestFlags;

		afTestFlags.AssignFlagsAllValues(OU_UINT32_MAX);

		if (afTestFlags.QueryFlagsAllValues() != (atomicord32)OU_UINT32_MAX)
		{
			break;
		}

		afTestFlags.AssignFlagsAllValues(0);

		if (afTestFlags.QueryFlagsAllValues() != 0)
		{
			break;
		}
	
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomicFlags_QueryFlagsAllValues()
{
	bool bResult = false;
	
	do
	{
		CAtomicFlags afTestFlags(g_aoTestValue32);
		
		if (afTestFlags.QueryFlagsAllValues() != g_aoTestValue32)
		{
			break;
		}

		// Double check to be sure ;-)
		if (afTestFlags.QueryFlagsAllValues() != g_aoTestValue32)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomicFlags_SetFlagsMaskValue()
{
	bool bResult = false;
	
	do
	{
		CAtomicFlags afTestFlags(g_aoTestValue32);

		afTestFlags.SetFlagsMaskValue(g_aoTestMask32, true);

		if (afTestFlags.QueryFlagsAllValues() != (atomicord32)(g_aoTestValue32 | g_aoTestMask32))
		{
			break;
		}

		afTestFlags.SetFlagsMaskValue(g_aoTestValue32, false);

		if (afTestFlags.QueryFlagsAllValues() != (atomicord32)(~g_aoTestValue32 & g_aoTestMask32))
		{
			break;
		}
		

		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomicFlags_SignalFlagsMaskValue()
{
	bool bResult = false;
	
	do
	{
		CAtomicFlags afTestFlags(g_aoTestValue32);
		
		afTestFlags.SignalFlagsMaskValue(g_aoTestMask32);
		
		if (afTestFlags.QueryFlagsAllValues() != (atomicord32)(g_aoTestValue32 | g_aoTestMask32))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomicFlags_DropFlagsMaskValue()
{
	bool bResult = false;
	
	do
	{
		CAtomicFlags afTestFlags(g_aoTestValue32);
		
		afTestFlags.DropFlagsMaskValue(g_aoTestMask32);
		
		if (afTestFlags.QueryFlagsAllValues() != (atomicord32)(g_aoTestValue32 & ~g_aoTestMask32))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomicFlags_ToggleSingleFlagValue()
{
	bool bResult = false;
	
	do
	{
		CAtomicFlags afTestFlags(g_aoTestValue32);
		
		bool bPreviousValue = afTestFlags.ToggleSingleFlagValue(g_aoTestBit32);
		
		if (bPreviousValue != ((g_aoTestValue32 & g_aoTestBit32) != 0) || afTestFlags.QueryFlagsAllValues() != (atomicord32)(g_aoTestValue32 ^ g_aoTestBit32))
		{
			break;
		}

		bool bAnotherPreviousValue = afTestFlags.ToggleSingleFlagValue(g_aoTestBit32);
		
		if (bAnotherPreviousValue == bPreviousValue || afTestFlags.QueryFlagsAllValues() != g_aoTestValue32)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomicFlags_ModifySingleFlagValue()
{
	bool bResult = false;
	
	do
	{
		CAtomicFlags afTestFlags(g_aoTestValue32);
		
		bool bFirstModification = afTestFlags.ModifySingleFlagValue(g_aoTestBit32, true);
		
		if (bFirstModification != ((g_aoTestValue32 & g_aoTestBit32) != g_aoTestBit32) || afTestFlags.QueryFlagsAllValues() != (atomicord32)(g_aoTestValue32 | g_aoTestBit32))
		{
			break;
		}
		
		bool bAnotherModification = afTestFlags.ModifySingleFlagValue(g_aoTestBit32, bFirstModification);
		
		if (bAnotherModification == bFirstModification || afTestFlags.QueryFlagsAllValues() != (bFirstModification ? (atomicord32)(g_aoTestValue32 | g_aoTestBit32) : (atomicord32)(g_aoTestValue32 & ~g_aoTestBit32)))
		{
			break;
		}

		bool bYetAnotherModification = afTestFlags.ModifySingleFlagValue(g_aoTestBit32, bAnotherModification);

		if (bYetAnotherModification != bAnotherModification || afTestFlags.QueryFlagsAllValues() != (bAnotherModification ? (atomicord32)(g_aoTestValue32 | g_aoTestBit32) : (atomicord32)(g_aoTestValue32 & ~g_aoTestBit32)))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomicFlags_AssignFlagsByMask()
{
	bool bResult = false;
	
	do
	{
		CAtomicFlags afTestFlags(g_aoTestValue32);

		atomicord32 aoPreviousFlags = afTestFlags.AssignFlagsByMask(g_aoTestMask32, g_aoTestMask32);

		const atomicord32 aoNewFlags = (g_aoTestValue32 & ~g_aoTestMask32) | g_aoTestMask32;

		if (aoPreviousFlags != g_aoTestValue32 || afTestFlags.QueryFlagsAllValues() != aoNewFlags)
		{
			break;
		}

		atomicord32 aoAnotherPreviousFlags = afTestFlags.AssignFlagsByMask(g_aoTestValue32, 0);

		const atomicord32 aoAnotherNewFlags = aoNewFlags & ~g_aoTestValue32;

		if (aoAnotherPreviousFlags != aoNewFlags || afTestFlags.QueryFlagsAllValues() != aoAnotherNewFlags)
		{
			break;
		}
	
		atomicord32 aoYetAnotherPreviousFlags = afTestFlags.AssignFlagsByMask(g_aoTestMask32, g_aoTestMask32 & g_aoTestValue32);
		OU_ASSERT((g_aoTestMask32 & g_aoTestValue32) != 0); // Test degeneration
		
		const atomicord32 aoYetAnotherNewFlags = (aoAnotherNewFlags & ~g_aoTestMask32) | (g_aoTestMask32 & g_aoTestValue32);
		OU_ASSERT(aoYetAnotherNewFlags != (atomicord32)OU_UINT32_MAX); // Test degeneration
		
		if (aoYetAnotherPreviousFlags != aoAnotherNewFlags || afTestFlags.QueryFlagsAllValues() != aoYetAnotherNewFlags)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomicFlags_AlterFlagsByMask()
{
	bool bResult = false;
	
	do
	{
		CAtomicFlags afTestFlags(g_aoTestValue32);
		
		bool bWasModification = afTestFlags.AlterFlagsByMask(g_aoTestMask32, g_aoTestMask32);
		
		const atomicord32 aoNewFlags = (g_aoTestValue32 & ~g_aoTestMask32) | g_aoTestMask32;
		
		if (bWasModification != ((g_aoTestValue32 & g_aoTestMask32) != g_aoTestMask32) || afTestFlags.QueryFlagsAllValues() != aoNewFlags)
		{
			break;
		}
		
		bool bWasAnotherModification = afTestFlags.AlterFlagsByMask(g_aoTestValue32, 0);
		
		const atomicord32 aoAnotherNewFlags = aoNewFlags & ~g_aoTestValue32;
		
		if (bWasAnotherModification != ((aoNewFlags & g_aoTestValue32) != 0) || afTestFlags.QueryFlagsAllValues() != aoAnotherNewFlags)
		{
			break;
		}
		
		bool bWasAnotherModificationRepeated = afTestFlags.AlterFlagsByMask(g_aoTestValue32, 0);
		
		if (bWasAnotherModificationRepeated || afTestFlags.QueryFlagsAllValues() != aoAnotherNewFlags)
		{
			break;
		}
		
		bool bWasYetAnotherModification = afTestFlags.AlterFlagsByMask(g_aoTestMask32, g_aoTestMask32 & g_aoTestValue32);
		OU_ASSERT((g_aoTestMask32 & g_aoTestValue32) != 0); // Test degeneration
		
		const atomicord32 aoYetAnotherNewFlags = (aoAnotherNewFlags & ~g_aoTestMask32) | (g_aoTestMask32 & g_aoTestValue32);
		OU_ASSERT(aoYetAnotherNewFlags != (atomicord32)OU_UINT32_MAX); // Test degeneration
		
		if (bWasYetAnotherModification != ((aoAnotherNewFlags & g_aoTestMask32) != (g_aoTestMask32 & g_aoTestValue32)) || afTestFlags.QueryFlagsAllValues() != aoYetAnotherNewFlags)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomicFlags_GetFlagsMaskValue()
{
	bool bResult = false;
	
	do
	{
		CAtomicFlags afTestFlags(g_aoTestValue32);
		
		if (afTestFlags.GetFlagsMaskValue(g_aoTestMask32) != ((g_aoTestValue32 & g_aoTestMask32) != 0))
		{
			break;
		}
		
		if (afTestFlags.GetFlagsMaskValue(~g_aoTestValue32))
		{
			break;
		}
		
		if (!afTestFlags.GetFlagsMaskValue(OU_UINT32_MAX))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomicFlags_QueryFlagsByMask()
{
	bool bResult = false;
	
	do
	{
		CAtomicFlags afTestFlags(g_aoTestValue32);
		
		if (afTestFlags.QueryFlagsByMask(g_aoTestMask32) != (atomicord32)(g_aoTestValue32 & g_aoTestMask32))
		{
			break;
		}
		
		if (afTestFlags.QueryFlagsByMask(0))
		{
			break;
		}
		
		if (afTestFlags.QueryFlagsByMask(OU_UINT32_MAX) != g_aoTestValue32)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomicFlags_OnlySignalSingleFlagOutOfMask()
{
	bool bResult = false;
	
	do
	{
		CAtomicFlags afTestFlags(g_aoTestValue32);
		OU_ASSERT(g_aoTestValue32 != 0); // Test degeneration
		
		if (afTestFlags.OnlySignalSingleFlagOutOfMask(OU_UINT32_MAX, g_aoTestBit32))
		{
			break;
		}

		if (afTestFlags.QueryFlagsAllValues() != g_aoTestValue32)
		{
			break;
		}
		
		afTestFlags.AssignFlagsAllValues(0);

		if (!afTestFlags.OnlySignalSingleFlagOutOfMask(g_aoTestBit32, g_aoTestBit32))
		{
			break;
		}
		
		if (afTestFlags.QueryFlagsAllValues() != g_aoTestBit32)
		{
			break;
		}
		
		if (afTestFlags.OnlySignalSingleFlagOutOfMask(g_aoTestBit32, g_aoTestBit32))
		{
			break;
		}
		
		if (afTestFlags.QueryFlagsAllValues() != g_aoTestBit32)
		{
			break;
		}
		
		if (afTestFlags.OnlySignalSingleFlagOutOfMask(OU_UINT32_MAX, g_aoTestAnotherBit32))
		{
			break;
		}
		
		if (afTestFlags.QueryFlagsAllValues() != g_aoTestBit32)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomicFlags_EnumSetEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CAtomicFlags afTestFlags;
		
		afTestFlags.EnumSetEnumeratedFlagValue(1, 0, OU_UINT32_BITS, true);

		if (afTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
	
		afTestFlags.EnumSetEnumeratedFlagValue(1, OU_UINT32_BITS - 1, OU_UINT32_BITS, true);
		
		if (afTestFlags.QueryFlagsAllValues() != (atomicord32)(OU_INT32_MIN + 1))
		{
			break;
		}
		
		afTestFlags.EnumSetEnumeratedFlagValue(1, 0, OU_UINT32_BITS, false);
		
		if (afTestFlags.QueryFlagsAllValues() != (atomicord32)OU_INT32_MIN)
		{
			break;
		}
		
		afTestFlags.EnumSetEnumeratedFlagValue(1, OU_UINT32_BITS - 1, OU_UINT32_BITS, false);
		
		if (afTestFlags.QueryFlagsAllValues() != 0)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomicFlags_EnumSignalEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CAtomicFlags afTestFlags;
		
		afTestFlags.EnumSignalEnumeratedFlagValue(1, 0, OU_UINT32_BITS);
		
		if (afTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		afTestFlags.EnumSignalEnumeratedFlagValue(1, OU_UINT32_BITS - 1, OU_UINT32_BITS);
		
		if (afTestFlags.QueryFlagsAllValues() != (atomicord32)(OU_INT32_MIN + 1))
		{
			break;
		}
		
		afTestFlags.EnumSignalEnumeratedFlagValue(1, 0, OU_UINT32_BITS);
		
		if (afTestFlags.QueryFlagsAllValues() != (atomicord32)(OU_INT32_MIN + 1))
		{
			break;
		}
		
		afTestFlags.EnumSignalEnumeratedFlagValue(1, OU_UINT32_BITS - 1, OU_UINT32_BITS);
		
		if (afTestFlags.QueryFlagsAllValues() != (atomicord32)(OU_INT32_MIN + 1))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomicFlags_EnumDropEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CAtomicFlags afTestFlags(OU_UINT32_MAX);
		
		afTestFlags.EnumDropEnumeratedFlagValue(1, 0, OU_UINT32_BITS);
		
		if (afTestFlags.QueryFlagsAllValues() != (atomicord32)(OU_UINT32_MAX ^ 1))
		{
			break;
		}
		
		afTestFlags.EnumDropEnumeratedFlagValue(1, OU_UINT32_BITS - 1, OU_UINT32_BITS);
		
		if (afTestFlags.QueryFlagsAllValues() != (atomicord32)~(OU_INT32_MIN + 1))
		{
			break;
		}
		
		afTestFlags.EnumDropEnumeratedFlagValue(1, 0, OU_UINT32_BITS);
		
		if (afTestFlags.QueryFlagsAllValues() != (atomicord32)~(OU_INT32_MIN + 1))
		{
			break;
		}
		
		afTestFlags.EnumDropEnumeratedFlagValue(1, OU_UINT32_BITS - 1, OU_UINT32_BITS);
		
		if (afTestFlags.QueryFlagsAllValues() != (atomicord32)~(OU_INT32_MIN + 1))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomicFlags_EnumToggleEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CAtomicFlags afTestFlags;
		
		bool bToggleFirstResult = afTestFlags.EnumToggleEnumeratedFlagValue(1, 0, OU_UINT32_BITS);
		
		if (bToggleFirstResult || afTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		bool bToggleSecondResult = afTestFlags.EnumToggleEnumeratedFlagValue(1, OU_UINT32_BITS - 1, OU_UINT32_BITS);
		
		if (bToggleSecondResult || afTestFlags.QueryFlagsAllValues() != (atomicord32)(OU_INT32_MIN + 1))
		{
			break;
		}
		
		bool bToggleThirdResult = afTestFlags.EnumToggleEnumeratedFlagValue(1, 0, OU_UINT32_BITS);
		
		if (!bToggleThirdResult || afTestFlags.QueryFlagsAllValues() != (atomicord32)OU_INT32_MIN)
		{
			break;
		}
		
		bool bToggleFourthResult = afTestFlags.EnumToggleEnumeratedFlagValue(1, OU_UINT32_BITS - 1, OU_UINT32_BITS);
		
		if (!bToggleFourthResult || afTestFlags.QueryFlagsAllValues() != 0)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomicFlags_EnumModifyEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CAtomicFlags afTestFlags;
		
		bool bModifyFirstResult = afTestFlags.EnumModifyEnumeratedFlagValue(1, 0, OU_UINT32_BITS, true);
		
		if (!bModifyFirstResult || afTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		bool bModifySecondResult = afTestFlags.EnumModifyEnumeratedFlagValue(1, OU_UINT32_BITS - 1, OU_UINT32_BITS, true);
		
		if (!bModifySecondResult || afTestFlags.QueryFlagsAllValues() != (atomicord32)(OU_INT32_MIN + 1))
		{
			break;
		}
		
		bool bModifyThirdResult = afTestFlags.EnumModifyEnumeratedFlagValue(1, 0, OU_UINT32_BITS, true);
		
		if (bModifyThirdResult || afTestFlags.QueryFlagsAllValues() != (atomicord32)(OU_INT32_MIN + 1))
		{
			break;
		}
		
		bool bModifyFourthResult = afTestFlags.EnumModifyEnumeratedFlagValue(1, OU_UINT32_BITS - 1, OU_UINT32_BITS, true);
		
		if (bModifyFourthResult || afTestFlags.QueryFlagsAllValues() != (atomicord32)(OU_INT32_MIN + 1))
		{
			break;
		}
		
		bool bModifyFifthResult = afTestFlags.EnumModifyEnumeratedFlagValue(1, 0, OU_UINT32_BITS, false);
		
		if (!bModifyFifthResult || afTestFlags.QueryFlagsAllValues() != (atomicord32)OU_INT32_MIN)
		{
			break;
		}
		
		bool bModifySixthResult = afTestFlags.EnumModifyEnumeratedFlagValue(1, OU_UINT32_BITS - 1, OU_UINT32_BITS, false);
		
		if (!bModifySixthResult || afTestFlags.QueryFlagsAllValues() != 0)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomicFlags_EnumSignalFirstEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CAtomicFlags afTestFlags;

		bool bFirstResult = afTestFlags.EnumSignalFirstEnumeratedFlagValue(1, 0, OU_UINT32_BITS);

		if (!bFirstResult || afTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		bool bSecondResult = afTestFlags.EnumSignalFirstEnumeratedFlagValue(1, 0, OU_UINT32_BITS);
		
		if (bSecondResult || afTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		bool bThirdResult = afTestFlags.EnumSignalFirstEnumeratedFlagValue(2, 0, OU_UINT32_BITS - 1);
		
		if (!bThirdResult || afTestFlags.QueryFlagsAllValues() != 3)
		{
			break;
		}
		
		bool bFourthResult = afTestFlags.EnumSignalFirstEnumeratedFlagValue(1, OU_UINT32_BITS - 1, OU_UINT32_BITS);
		
		if (bFourthResult || afTestFlags.QueryFlagsAllValues() != (atomicord32)(OU_INT32_MIN + 3))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomicFlags_EnumSignalLastEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CAtomicFlags afTestFlags;
		
		bool bFirstResult = afTestFlags.EnumSignalLastEnumeratedFlagValue(1, 0, 1);
		
		if (!bFirstResult || afTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		bool bSecondResult = afTestFlags.EnumSignalLastEnumeratedFlagValue(1, 0, 1);
		
		if (bSecondResult || afTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		bool bThirdResult = afTestFlags.EnumSignalLastEnumeratedFlagValue(1, 1, 2);
		
		if (!bThirdResult || afTestFlags.QueryFlagsAllValues() != 3)
		{
			break;
		}
		
		bool bFourthResult = afTestFlags.EnumSignalLastEnumeratedFlagValue(1, OU_UINT32_BITS - 1, OU_UINT32_BITS);
		
		if (bFourthResult || afTestFlags.QueryFlagsAllValues() != (atomicord32)(OU_INT32_MIN + 3))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomicFlags_EnumGetEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CAtomicFlags afTestFlags((atomicord32)(OU_INT32_MIN + 1));
		
		if (!afTestFlags.EnumGetEnumeratedFlagValue(1, 0, OU_UINT32_BITS))
		{
			break;
		}
	
		if (afTestFlags.EnumGetEnumeratedFlagValue(2, 0, OU_UINT32_BITS - 1))
		{
			break;
		}
		
		if (afTestFlags.EnumGetEnumeratedFlagValue(1, 1, OU_UINT32_BITS - 1))
		{
			break;
		}
		
		if (!afTestFlags.EnumGetEnumeratedFlagValue(1, OU_UINT32_BITS - 1, OU_UINT32_BITS))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomicFlags_EnumFindFirstEnumeratedFlag()
{
	bool bResult = false;
	
	do
	{
		CAtomicFlags afTestFlags((atomicord32)(OU_INT32_MIN + 1));

		unsigned int uiFirstResult = afTestFlags.EnumFindFirstEnumeratedFlag(1, OU_UINT32_BITS);
		if (uiFirstResult != 0)
		{
			break;
		}

		unsigned int uiSecondResult = afTestFlags.EnumFindFirstEnumeratedFlag(2, OU_UINT32_BITS - 1);
		if (uiSecondResult != OU_UINT32_BITS - 2)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomicFlags_EnumAllSignalEnumeratedFlags()
{
	bool bResult = false;
	
	do
	{
		CAtomicFlags afTestFlags;
		
		afTestFlags.EnumAllSignalEnumeratedFlags(1, 1);

		if (afTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}

		afTestFlags.EnumAllSignalEnumeratedFlags(4, OU_UINT32_BITS - 2);

		if (afTestFlags.QueryFlagsAllValues() != (atomicord32)(OU_UINT32_MAX ^ 2))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomicFlags_EnumAllDropEnumeratedFlags()
{
	bool bResult = false;
	
	do
	{
		CAtomicFlags afTestFlags(OU_UINT32_MAX);
		
		afTestFlags.EnumAllDropEnumeratedFlags(1, 1);
		
		if (afTestFlags.QueryFlagsAllValues() != (atomicord32)(OU_UINT32_MAX ^ 1))
		{
			break;
		}
		
		afTestFlags.EnumAllDropEnumeratedFlags(4, OU_UINT32_BITS - 2);
		
		if (afTestFlags.QueryFlagsAllValues() != 2)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomicFlags_EnumAllQueryEnumeratedFlags()
{
	bool bResult = false;
	
	do
	{
		CAtomicFlags afTestFlags((atomicord32)(OU_INT32_MIN + 1));
		
		atomicord32 aoFirstResult = afTestFlags.EnumAllQueryEnumeratedFlags(1, OU_UINT32_BITS);
		if (aoFirstResult != (atomicord32)(OU_INT32_MIN + 1))
		{
			break;
		}
		
		atomicord32 aoSecondResult = afTestFlags.EnumAllQueryEnumeratedFlags(2, OU_UINT32_BITS - 1);
		if (aoSecondResult != (atomicord32)(OU_INT32_MIN))
		{
			break;
		}
		
		atomicord32 aoThirdResult = afTestFlags.EnumAllQueryEnumeratedFlags(2, OU_UINT32_BITS - 2);
		if (aoThirdResult != 0)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomicFlags_EnumAnyGetEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CAtomicFlags afTestFlags((atomicord32)(OU_INT32_MIN + 1));
		
		bool bFirstResult = afTestFlags.EnumAnyGetEnumeratedFlagValue(1, OU_UINT32_BITS);
		if (!bFirstResult)
		{
			break;
		}
		
		bool bSecondResult = afTestFlags.EnumAnyGetEnumeratedFlagValue(2, OU_UINT32_BITS - 1);
		if (!bSecondResult)
		{
			break;
		}
		
		bool bThirdResult = afTestFlags.EnumAnyGetEnumeratedFlagValue(2, OU_UINT32_BITS - 2);
		if (bThirdResult)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomicFlags_StoreFlagsEnumeratedValue()
{
	bool bResult = false;
	
	do
	{
		CAtomicFlags afTestFlags;

		afTestFlags.StoreFlagsEnumeratedValue(0x03, 1, 2);

		if (afTestFlags.QueryFlagsAllValues() != (atomicord32)(2 << 1))
		{
			break;
		}
	
		afTestFlags.StoreFlagsEnumeratedValue(0x03, OU_UINT32_BITS - 2, 3);
		
		if (afTestFlags.QueryFlagsAllValues() != ((atomicord32)(2 << 1) | (atomicord32)(OU_INT32_MIN | (OU_INT32_MIN >> 1))))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestAtomicFlags_RetrieveFlagsEnumeratedValue()
{
	bool bResult = false;
	
	do
	{
		CAtomicFlags afTestFlags((atomicord32)(OU_INT32_MIN + 1));
		
		unsigned int aoFirstResult = afTestFlags.RetrieveFlagsEnumeratedValue(0x3, 1);
		if (aoFirstResult != 0)
		{
			break;
		}
		
		unsigned int aoSecondResult = afTestFlags.RetrieveFlagsEnumeratedValue(0x3, OU_UINT32_BITS - 2);
		if (aoSecondResult != 2)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}


enum EOUATOMICFLAGSFEATURE
{
	OAF__MIN,
		
	OAF_CONSTRUCTORS = OAF__MIN,
	OAF_ASSIGNFLAGSALLVALUES,
	OAF_QUERYFLAGSALLVALUES,
	OAF_SETFLAGSMASKVALUE,
	OAF_SIGNALFLAGSMASKVALUE,
	OAF_DROPFLAGSMASKVALUE,
	OAF_TOGGLESINGLEFLAGVALUE,
	OAF_MODIFYSINGLEFLAGVALUE,
	OAF_ASSIGNFLAGSBYMASK,
	OAF_ALTERFLAGSBYMASK,
	OAF_GETFLAGSMASKVALUE,
	OAF_QUERYFLAGSBYMASK,
	OAF_ONLYSIGNALSINGLEFLAGOUTOFMASK,
	OAF_ENUMSETENUMERATEDFLAGVALUE,
	OAF_ENUMSIGNALENUMERATEDFLAGVALUE,
	OAF_ENUMDROPENUMERATEDFLAGVALUE,
	OAF_ENUMTOGGLEENUMERATEDFLAGVALUE,
	OAF_ENUMMODIFYENUMERATEDFLAGVALUE,
	OAF_ENUMSIGNALFIRSTENUMERATEDFLAGVALUE,
	OAF_ENUMSIGNALLASTENUMERATEDFLAGVALUE,
	OAF_ENUMGETENUMERATEDFLAGVALUE,
	OAF_ENUMFINDFIRSTENUMERATEDFLAG,
	OAF_ENUMALLSIGNALENUMERATEDFLAGS,
	OAF_ENUMALLDROPENUMERATEDFLAGS,
	OAF_ENUMALLQUERYENUMERATEDFLAGS,
	OAF_ENUMANYGETENUMERATEDFLAGVALUE,
	OAF_STOREFLAGSENUMERATEDVALUE,
	OAF_RETRIEVEFLAGSENUMERATEDVALUE,

	OAF__MAX,
};

template<>
CFeatureTestProcedure const CEnumUnsortedElementArray<EOUATOMICFLAGSFEATURE, OAF__MAX, CFeatureTestProcedure>::m_aetElementArray[] =
{
	&TestAtomicFlags_Constructors, // OAF_CONSTRUCTORS
	&TestAtomicFlags_AssignFlagsAllValues, // OAF_ASSIGNFLAGSALLVALUES,
	&TestAtomicFlags_QueryFlagsAllValues, // OAF_QUERYFLAGSALLVALUES,
	&TestAtomicFlags_SetFlagsMaskValue, // OAF_SETFLAGSMASKVALUE,
	&TestAtomicFlags_SignalFlagsMaskValue, // OAF_SIGNALFLAGSMASKVALUE,
	&TestAtomicFlags_DropFlagsMaskValue, // OAF_DROPFLAGSMASKVALUE,
	&TestAtomicFlags_ToggleSingleFlagValue, // OAF_TOGGLESINGLEFLAGVALUE,
	&TestAtomicFlags_ModifySingleFlagValue, // OAF_MODIFYSINGLEFLAGVALUE,
	&TestAtomicFlags_AssignFlagsByMask, // OAF_ASSIGNFLAGSBYMASK,
	&TestAtomicFlags_AlterFlagsByMask, // OAF_ALTERFLAGSBYMASK,
	&TestAtomicFlags_GetFlagsMaskValue, // OAF_GETFLAGSMASKVALUE,
	&TestAtomicFlags_QueryFlagsByMask, // OAF_QUERYFLAGSBYMASK,
	&TestAtomicFlags_OnlySignalSingleFlagOutOfMask, // OAF_ONLYSIGNALSINGLEFLAGOUTOFMASK,
	&TestAtomicFlags_EnumSetEnumeratedFlagValue, // OAF_ENUMSETENUMERATEDFLAGVALUE,
	&TestAtomicFlags_EnumSignalEnumeratedFlagValue, // OAF_ENUMSIGNALENUMERATEDFLAGVALUE,
	&TestAtomicFlags_EnumDropEnumeratedFlagValue, // OAF_ENUMDROPENUMERATEDFLAGVALUE,
	&TestAtomicFlags_EnumToggleEnumeratedFlagValue, // OAF_ENUMTOGGLEENUMERATEDFLAGVALUE,
	&TestAtomicFlags_EnumModifyEnumeratedFlagValue, // OAF_ENUMMODIFYENUMERATEDFLAGVALUE,
	&TestAtomicFlags_EnumSignalFirstEnumeratedFlagValue, // OAF_ENUMSIGNALFIRSTENUMERATEDFLAGVALUE,
	&TestAtomicFlags_EnumSignalLastEnumeratedFlagValue, // OAF_ENUMSIGNALLASTENUMERATEDFLAGVALUE,
	&TestAtomicFlags_EnumGetEnumeratedFlagValue, // OAF_ENUMGETENUMERATEDFLAGVALUE,
	&TestAtomicFlags_EnumFindFirstEnumeratedFlag, // OAF_ENUMFINDFIRSTENUMERATEDFLAG,
	&TestAtomicFlags_EnumAllSignalEnumeratedFlags, // OAF_ENUMALLSIGNALENUMERATEDFLAGS,
	&TestAtomicFlags_EnumAllDropEnumeratedFlags, // OAF_ENUMALLDROPENUMERATEDFLAGS,
	&TestAtomicFlags_EnumAllQueryEnumeratedFlags, // OAF_ENUMALLQUERYENUMERATEDFLAGS,
	&TestAtomicFlags_EnumAnyGetEnumeratedFlagValue, // OAF_ENUMANYGETENUMERATEDFLAGVALUE,
	&TestAtomicFlags_StoreFlagsEnumeratedValue, // OAF_STOREFLAGSENUMERATEDVALUE,
	&TestAtomicFlags_RetrieveFlagsEnumeratedValue, // OAF_RETRIEVEFLAGSENUMERATEDVALUE,
};
static const CEnumUnsortedElementArray<EOUATOMICFLAGSFEATURE, OAF__MAX, CFeatureTestProcedure> g_afnAtomicFlagsFeatureTestProcedures;

template<>
const char *const CEnumUnsortedElementArray<EOUATOMICFLAGSFEATURE, OAF__MAX, const char *>::m_aetElementArray[] =
{
	"Constructors", // OAF_CONSTRUCTORS
	"AssignFlagsAllValues", // OAF_ASSIGNFLAGSALLVALUES,
	"QueryFlagsAllValues", // OAF_QUERYFLAGSALLVALUES,
	"SetFlagsMaskValue", // OAF_SETFLAGSMASKVALUE,
	"SignalFlagsMaskValue", // OAF_SIGNALFLAGSMASKVALUE,
	"DropFlagsMaskValue", // OAF_DROPFLAGSMASKVALUE,
	"ToggleSingleFlagValue", // OAF_TOGGLESINGLEFLAGVALUE,
	"ModifySingleFlagValue", // OAF_MODIFYSINGLEFLAGVALUE,
	"AssignFlagsByMask", // OAF_ASSIGNFLAGSBYMASK,
	"AlterFlagsByMask", // OAF_ALTERFLAGSBYMASK,
	"GetFlagsMaskValue", // OAF_GETFLAGSMASKVALUE,
	"QueryFlagsByMask", // OAF_QUERYFLAGSBYMASK,
	"OnlySignalSingleFlagOutOfMask", // OAF_ONLYSIGNALSINGLEFLAGOUTOFMASK,
	"EnumSetEnumeratedFlagValue", // OAF_ENUMSETENUMERATEDFLAGVALUE,
	"EnumSignalEnumeratedFlagValue", // OAF_ENUMSIGNALENUMERATEDFLAGVALUE,
	"EnumDropEnumeratedFlagValue", // OAF_ENUMDROPENUMERATEDFLAGVALUE,
	"EnumToggleEnumeratedFlagValue", // OAF_ENUMTOGGLEENUMERATEDFLAGVALUE,
	"EnumModifyEnumeratedFlagValue", // OAF_ENUMMODIFYENUMERATEDFLAGVALUE,
	"EnumSignalFirstEnumeratedFlagValue", // OAF_ENUMSIGNALFIRSTENUMERATEDFLAGVALUE,
	"EnumSignalLastEnumeratedFlagValue", // OAF_ENUMSIGNALLASTENUMERATEDFLAGVALUE,
	"EnumGetEnumeratedFlagValue", // OAF_ENUMGETENUMERATEDFLAGVALUE,
	"EnumFindFirstEnumeratedFlag", // OAF_ENUMFINDFIRSTENUMERATEDFLAG,
	"EnumAllSignalEnumeratedFlags", // OAF_ENUMALLSIGNALENUMERATEDFLAGS,
	"EnumAllDropEnumeratedFlags", // OAF_ENUMALLDROPENUMERATEDFLAGS,
	"EnumAllQueryEnumeratedFlags", // OAF_ENUMALLQUERYENUMERATEDFLAGS,
	"EnumAnyGetEnumeratedFlagValue", // OAF_ENUMANYGETENUMERATEDFLAGVALUE,
	"StoreFlagsEnumeratedValue", // OAF_STOREFLAGSENUMERATEDVALUE,
	"RetrieveFlagsEnumeratedValue", // OAF_RETRIEVEFLAGSENUMERATEDVALUE,
};
static const CEnumUnsortedElementArray<EOUATOMICFLAGSFEATURE, OAF__MAX, const char *> g_aszAtomicFlagsFeatureTestNames;


bool TestAtomicFlags(unsigned int &nOutSuccessCount, unsigned int &nOutTestCount)
{
	bool bResult = false;
	nOutSuccessCount = 0;
	nOutTestCount = OAF__MAX;
	
	bool bAPIInitialized = false;
	
	do
	{
		if (!InitializeAtomicAPI())
		{
			break;
		}
		
		bAPIInitialized = true;
		
		if (!TestSubsystem(nOutSuccessCount, nOutTestCount, OAF__MAX, g_aszAtomicFlagsFeatureTestNames.GetStoragePointer(), g_afnAtomicFlagsFeatureTestProcedures.GetStoragePointer()))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	if (bAPIInitialized)
	{
		FinalizeAtomicAPI();
	}
	
	return bResult;
}


#endif // #if _OU_FEATURE_SET >= _OU_FEATURE_SET_ATOMICS


//////////////////////////////////////////////////////////////////////////

typedef CSimpleFlagsTemplate<uint64ou> CSimpleFlags64;

const uint64ou g_uiTestValue64 = ((uint64ou)0xA5A5A5A5 << 32) | 0xA5A5A5A5;
const uint64ou g_uiTestMask64 = ((uint64ou)0xC6C6C6C6 << 32) | 0xC6C6C6C6;
const uint64ou g_uiTestBit64 = (uint64ou)OU_INT64_MIN;
const uint64ou g_uiTestAnotherBit64 = (uint64ou)((uint64ou)OU_INT64_MIN >> 1);


bool TestSimpleFlags64_Constructors()
{
	bool bResult = false;
	
	do
	{
		if (sizeof(CSimpleFlags64::value_type) != sizeof(uint64ou) || sizeof(CSimpleFlags64) != sizeof(uint64ou))
		{
			break;
		}

		CSimpleFlags64 sfEmptyFlags;

		if (sfEmptyFlags.QueryFlagsAllValues())
		{
			break;
		}

		CSimpleFlags64 sfFullFlags(OU_UINT64_MAX);
		
		if (sfFullFlags.QueryFlagsAllValues() != OU_UINT64_MAX)
		{
			break;
		}

		CSimpleFlags64 sfCopyOfFullFlags(sfFullFlags);

		if (sfCopyOfFullFlags.QueryFlagsAllValues() != OU_UINT64_MAX)
		{
			break;
		}
	
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags64_AssignFlagsAllValues()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags64 sfTestFlags;

		sfTestFlags.AssignFlagsAllValues(OU_UINT64_MAX);

		if (sfTestFlags.QueryFlagsAllValues() != OU_UINT64_MAX)
		{
			break;
		}

		sfTestFlags.AssignFlagsAllValues(0);

		if (sfTestFlags.QueryFlagsAllValues() != 0)
		{
			break;
		}
	
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags64_QueryFlagsAllValues()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags64 sfTestFlags(g_uiTestValue64);
		
		if (sfTestFlags.QueryFlagsAllValues() != g_uiTestValue64)
		{
			break;
		}

		// Double check to be sure ;-)
		if (sfTestFlags.QueryFlagsAllValues() != g_uiTestValue64)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags64_SetFlagsMaskValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags64 sfTestFlags(g_uiTestValue64);

		sfTestFlags.SetFlagsMaskValue(g_uiTestMask64, true);

		if (sfTestFlags.QueryFlagsAllValues() != (uint64ou)(g_uiTestValue64 | g_uiTestMask64))
		{
			break;
		}

		sfTestFlags.SetFlagsMaskValue(g_uiTestValue64, false);

		if (sfTestFlags.QueryFlagsAllValues() != (uint64ou)(~g_uiTestValue64 & g_uiTestMask64))
		{
			break;
		}
		

		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags64_SignalFlagsMaskValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags64 sfTestFlags(g_uiTestValue64);
		
		sfTestFlags.SignalFlagsMaskValue(g_uiTestMask64);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint64ou)(g_uiTestValue64 | g_uiTestMask64))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags64_DropFlagsMaskValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags64 sfTestFlags(g_uiTestValue64);
		
		sfTestFlags.DropFlagsMaskValue(g_uiTestMask64);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint64ou)(g_uiTestValue64 & ~g_uiTestMask64))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags64_ToggleSingleFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags64 sfTestFlags(g_uiTestValue64);
		
		bool bPreviousValue = sfTestFlags.ToggleSingleFlagValue(g_uiTestBit64);
		
		if (bPreviousValue != ((g_uiTestValue64 & g_uiTestBit64) != 0) || sfTestFlags.QueryFlagsAllValues() != (uint64ou)(g_uiTestValue64 ^ g_uiTestBit64))
		{
			break;
		}

		bool bAnotherPreviousValue = sfTestFlags.ToggleSingleFlagValue(g_uiTestBit64);
		
		if (bAnotherPreviousValue == bPreviousValue || sfTestFlags.QueryFlagsAllValues() != g_uiTestValue64)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags64_ModifySingleFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags64 sfTestFlags(g_uiTestValue64);
		
		bool bFirstModification = sfTestFlags.ModifySingleFlagValue(g_uiTestBit64, true);
		
		if (bFirstModification != ((g_uiTestValue64 & g_uiTestBit64) != g_uiTestBit64) || sfTestFlags.QueryFlagsAllValues() != (uint64ou)(g_uiTestValue64 | g_uiTestBit64))
		{
			break;
		}
		
		bool bAnotherModification = sfTestFlags.ModifySingleFlagValue(g_uiTestBit64, bFirstModification);
		
		if (bAnotherModification == bFirstModification || sfTestFlags.QueryFlagsAllValues() != (bFirstModification ? (uint64ou)(g_uiTestValue64 | g_uiTestBit64) : (uint64ou)(g_uiTestValue64 & ~g_uiTestBit64)))
		{
			break;
		}

		bool bYetAnotherModification = sfTestFlags.ModifySingleFlagValue(g_uiTestBit64, bAnotherModification);

		if (bYetAnotherModification != bAnotherModification || sfTestFlags.QueryFlagsAllValues() != (bAnotherModification ? (uint64ou)(g_uiTestValue64 | g_uiTestBit64) : (uint64ou)(g_uiTestValue64 & ~g_uiTestBit64)))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags64_AssignFlagsByMask()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags64 sfTestFlags(g_uiTestValue64);

		uint64ou uiPreviousFlags = sfTestFlags.AssignFlagsByMask(g_uiTestMask64, g_uiTestMask64);

		const uint64ou uiNewFlags = (g_uiTestValue64 & ~g_uiTestMask64) | g_uiTestMask64;

		if (uiPreviousFlags != g_uiTestValue64 || sfTestFlags.QueryFlagsAllValues() != uiNewFlags)
		{
			break;
		}

		uint64ou uiAnotherPreviousFlags = sfTestFlags.AssignFlagsByMask(g_uiTestValue64, 0);

		const uint64ou uiAnotherNewFlags = uiNewFlags & ~g_uiTestValue64;

		if (uiAnotherPreviousFlags != uiNewFlags || sfTestFlags.QueryFlagsAllValues() != uiAnotherNewFlags)
		{
			break;
		}
	
		uint64ou uiYetAnotherPreviousFlags = sfTestFlags.AssignFlagsByMask(g_uiTestMask64, g_uiTestMask64 & g_uiTestValue64);
		OU_ASSERT((g_uiTestMask64 & g_uiTestValue64) != 0); // Test degeneration
		
		const uint64ou uiYetAnotherNewFlags = (uiAnotherNewFlags & ~g_uiTestMask64) | (g_uiTestMask64 & g_uiTestValue64);
		OU_ASSERT(uiYetAnotherNewFlags != OU_UINT64_MAX); // Test degeneration
		
		if (uiYetAnotherPreviousFlags != uiAnotherNewFlags || sfTestFlags.QueryFlagsAllValues() != uiYetAnotherNewFlags)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags64_AlterFlagsByMask()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags64 sfTestFlags(g_uiTestValue64);
		
		bool bWasModification = sfTestFlags.AlterFlagsByMask(g_uiTestMask64, g_uiTestMask64);
		
		const uint64ou uiNewFlags = (g_uiTestValue64 & ~g_uiTestMask64) | g_uiTestMask64;
		
		if (bWasModification != ((g_uiTestValue64 & g_uiTestMask64) != g_uiTestMask64) || sfTestFlags.QueryFlagsAllValues() != uiNewFlags)
		{
			break;
		}
		
		bool bWasAnotherModification = sfTestFlags.AlterFlagsByMask(g_uiTestValue64, 0);
		
		const uint64ou uiAnotherNewFlags = uiNewFlags & ~g_uiTestValue64;
		
		if (bWasAnotherModification != ((uiNewFlags & g_uiTestValue64) != 0) || sfTestFlags.QueryFlagsAllValues() != uiAnotherNewFlags)
		{
			break;
		}
		
		bool bWasAnotherModificationRepeated = sfTestFlags.AlterFlagsByMask(g_uiTestValue64, 0);
		
		if (bWasAnotherModificationRepeated || sfTestFlags.QueryFlagsAllValues() != uiAnotherNewFlags)
		{
			break;
		}
		
		bool bWasYetAnotherModification = sfTestFlags.AlterFlagsByMask(g_uiTestMask64, g_uiTestMask64 & g_uiTestValue64);
		OU_ASSERT((g_uiTestMask64 & g_uiTestValue64) != 0); // Test degeneration
		
		const uint64ou uiYetAnotherNewFlags = (uiAnotherNewFlags & ~g_uiTestMask64) | (g_uiTestMask64 & g_uiTestValue64);
		OU_ASSERT(uiYetAnotherNewFlags != OU_UINT64_MAX); // Test degeneration
		
		if (bWasYetAnotherModification != ((uiAnotherNewFlags & g_uiTestMask64) != (g_uiTestMask64 & g_uiTestValue64)) || sfTestFlags.QueryFlagsAllValues() != uiYetAnotherNewFlags)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags64_GetFlagsMaskValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags64 sfTestFlags(g_uiTestValue64);
		
		if (sfTestFlags.GetFlagsMaskValue(g_uiTestMask64) != ((g_uiTestValue64 & g_uiTestMask64) != 0))
		{
			break;
		}
		
		if (sfTestFlags.GetFlagsMaskValue(~g_uiTestValue64))
		{
			break;
		}
		
		if (!sfTestFlags.GetFlagsMaskValue(OU_UINT64_MAX))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags64_QueryFlagsByMask()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags64 sfTestFlags(g_uiTestValue64);
		
		if (sfTestFlags.QueryFlagsByMask(g_uiTestMask64) != (uint64ou)(g_uiTestValue64 & g_uiTestMask64))
		{
			break;
		}
		
		if (sfTestFlags.QueryFlagsByMask(0))
		{
			break;
		}
		
		if (sfTestFlags.QueryFlagsByMask(OU_UINT64_MAX) != g_uiTestValue64)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags64_OnlySignalSingleFlagOutOfMask()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags64 sfTestFlags(g_uiTestValue64);
		OU_ASSERT(g_uiTestValue64 != 0); // Test degeneration
		
		if (sfTestFlags.OnlySignalSingleFlagOutOfMask(OU_UINT64_MAX, g_uiTestBit64))
		{
			break;
		}

		if (sfTestFlags.QueryFlagsAllValues() != g_uiTestValue64)
		{
			break;
		}
		
		sfTestFlags.AssignFlagsAllValues(0);

		if (!sfTestFlags.OnlySignalSingleFlagOutOfMask(g_uiTestBit64, g_uiTestBit64))
		{
			break;
		}
		
		if (sfTestFlags.QueryFlagsAllValues() != g_uiTestBit64)
		{
			break;
		}
		
		if (sfTestFlags.OnlySignalSingleFlagOutOfMask(g_uiTestBit64, g_uiTestBit64))
		{
			break;
		}
		
		if (sfTestFlags.QueryFlagsAllValues() != g_uiTestBit64)
		{
			break;
		}
		
		if (sfTestFlags.OnlySignalSingleFlagOutOfMask(OU_UINT64_MAX, g_uiTestAnotherBit64))
		{
			break;
		}
		
		if (sfTestFlags.QueryFlagsAllValues() != g_uiTestBit64)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags64_EnumSetEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags64 sfTestFlags;
		
		sfTestFlags.EnumSetEnumeratedFlagValue(1, 0, OU_UINT64_BITS, true);

		if (sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
	
		sfTestFlags.EnumSetEnumeratedFlagValue(1, OU_UINT64_BITS - 1, OU_UINT64_BITS, true);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint64ou)(OU_INT64_MIN + 1))
		{
			break;
		}
		
		sfTestFlags.EnumSetEnumeratedFlagValue(1, 0, OU_UINT64_BITS, false);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint64ou)OU_INT64_MIN)
		{
			break;
		}
		
		sfTestFlags.EnumSetEnumeratedFlagValue(1, OU_UINT64_BITS - 1, OU_UINT64_BITS, false);
		
		if (sfTestFlags.QueryFlagsAllValues() != 0)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags64_EnumSignalEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags64 sfTestFlags;
		
		sfTestFlags.EnumSignalEnumeratedFlagValue(1, 0, OU_UINT64_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		sfTestFlags.EnumSignalEnumeratedFlagValue(1, OU_UINT64_BITS - 1, OU_UINT64_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint64ou)(OU_INT64_MIN + 1))
		{
			break;
		}
		
		sfTestFlags.EnumSignalEnumeratedFlagValue(1, 0, OU_UINT64_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint64ou)(OU_INT64_MIN + 1))
		{
			break;
		}
		
		sfTestFlags.EnumSignalEnumeratedFlagValue(1, OU_UINT64_BITS - 1, OU_UINT64_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint64ou)(OU_INT64_MIN + 1))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags64_EnumDropEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags64 sfTestFlags(OU_UINT64_MAX);
		
		sfTestFlags.EnumDropEnumeratedFlagValue(1, 0, OU_UINT64_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != (OU_UINT64_MAX ^ 1))
		{
			break;
		}
		
		sfTestFlags.EnumDropEnumeratedFlagValue(1, OU_UINT64_BITS - 1, OU_UINT64_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint64ou)~(OU_INT64_MIN + 1))
		{
			break;
		}
		
		sfTestFlags.EnumDropEnumeratedFlagValue(1, 0, OU_UINT64_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint64ou)~(OU_INT64_MIN + 1))
		{
			break;
		}
		
		sfTestFlags.EnumDropEnumeratedFlagValue(1, OU_UINT64_BITS - 1, OU_UINT64_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint64ou)~(OU_INT64_MIN + 1))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags64_EnumToggleEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags64 sfTestFlags;
		
		bool bToggleFirstResult = sfTestFlags.EnumToggleEnumeratedFlagValue(1, 0, OU_UINT64_BITS);
		
		if (bToggleFirstResult || sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		bool bToggleSecondResult = sfTestFlags.EnumToggleEnumeratedFlagValue(1, OU_UINT64_BITS - 1, OU_UINT64_BITS);
		
		if (bToggleSecondResult || sfTestFlags.QueryFlagsAllValues() != (uint64ou)(OU_INT64_MIN + 1))
		{
			break;
		}
		
		bool bToggleThirdResult = sfTestFlags.EnumToggleEnumeratedFlagValue(1, 0, OU_UINT64_BITS);
		
		if (!bToggleThirdResult || sfTestFlags.QueryFlagsAllValues() != (uint64ou)OU_INT64_MIN)
		{
			break;
		}
		
		bool bToggleFourthResult = sfTestFlags.EnumToggleEnumeratedFlagValue(1, OU_UINT64_BITS - 1, OU_UINT64_BITS);
		
		if (!bToggleFourthResult || sfTestFlags.QueryFlagsAllValues() != 0)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags64_EnumModifyEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags64 sfTestFlags;
		
		bool bModifyFirstResult = sfTestFlags.EnumModifyEnumeratedFlagValue(1, 0, OU_UINT64_BITS, true);
		
		if (!bModifyFirstResult || sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		bool bModifySecondResult = sfTestFlags.EnumModifyEnumeratedFlagValue(1, OU_UINT64_BITS - 1, OU_UINT64_BITS, true);
		
		if (!bModifySecondResult || sfTestFlags.QueryFlagsAllValues() != (uint64ou)(OU_INT64_MIN + 1))
		{
			break;
		}
		
		bool bModifyThirdResult = sfTestFlags.EnumModifyEnumeratedFlagValue(1, 0, OU_UINT64_BITS, true);
		
		if (bModifyThirdResult || sfTestFlags.QueryFlagsAllValues() != (uint64ou)(OU_INT64_MIN + 1))
		{
			break;
		}
		
		bool bModifyFourthResult = sfTestFlags.EnumModifyEnumeratedFlagValue(1, OU_UINT64_BITS - 1, OU_UINT64_BITS, true);
		
		if (bModifyFourthResult || sfTestFlags.QueryFlagsAllValues() != (uint64ou)(OU_INT64_MIN + 1))
		{
			break;
		}
		
		bool bModifyFifthResult = sfTestFlags.EnumModifyEnumeratedFlagValue(1, 0, OU_UINT64_BITS, false);
		
		if (!bModifyFifthResult || sfTestFlags.QueryFlagsAllValues() != (uint64ou)OU_INT64_MIN)
		{
			break;
		}
		
		bool bModifySixthResult = sfTestFlags.EnumModifyEnumeratedFlagValue(1, OU_UINT64_BITS - 1, OU_UINT64_BITS, false);
		
		if (!bModifySixthResult || sfTestFlags.QueryFlagsAllValues() != 0)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags64_EnumSignalFirstEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags64 sfTestFlags;

		bool bFirstResult = sfTestFlags.EnumSignalFirstEnumeratedFlagValue(1, 0, OU_UINT64_BITS);

		if (!bFirstResult || sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		bool bSecondResult = sfTestFlags.EnumSignalFirstEnumeratedFlagValue(1, 0, OU_UINT64_BITS);
		
		if (bSecondResult || sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		bool bThirdResult = sfTestFlags.EnumSignalFirstEnumeratedFlagValue(2, 0, OU_UINT64_BITS - 1);
		
		if (!bThirdResult || sfTestFlags.QueryFlagsAllValues() != 3)
		{
			break;
		}
		
		bool bFourthResult = sfTestFlags.EnumSignalFirstEnumeratedFlagValue(1, OU_UINT64_BITS - 1, OU_UINT64_BITS);
		
		if (bFourthResult || sfTestFlags.QueryFlagsAllValues() != (uint64ou)(OU_INT64_MIN + 3))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags64_EnumSignalLastEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags64 sfTestFlags;
		
		bool bFirstResult = sfTestFlags.EnumSignalLastEnumeratedFlagValue(1, 0, 1);
		
		if (!bFirstResult || sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		bool bSecondResult = sfTestFlags.EnumSignalLastEnumeratedFlagValue(1, 0, 1);
		
		if (bSecondResult || sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		bool bThirdResult = sfTestFlags.EnumSignalLastEnumeratedFlagValue(1, 1, 2);
		
		if (!bThirdResult || sfTestFlags.QueryFlagsAllValues() != 3)
		{
			break;
		}
		
		bool bFourthResult = sfTestFlags.EnumSignalLastEnumeratedFlagValue(1, OU_UINT64_BITS - 1, OU_UINT64_BITS);
		
		if (bFourthResult || sfTestFlags.QueryFlagsAllValues() != (uint64ou)(OU_INT64_MIN + 3))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags64_EnumGetEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags64 sfTestFlags((uint64ou)(OU_INT64_MIN + 1));
		
		if (!sfTestFlags.EnumGetEnumeratedFlagValue(1, 0, OU_UINT64_BITS))
		{
			break;
		}
	
		if (sfTestFlags.EnumGetEnumeratedFlagValue(2, 0, OU_UINT64_BITS - 1))
		{
			break;
		}
		
		if (sfTestFlags.EnumGetEnumeratedFlagValue(1, 1, OU_UINT64_BITS - 1))
		{
			break;
		}
		
		if (!sfTestFlags.EnumGetEnumeratedFlagValue(1, OU_UINT64_BITS - 1, OU_UINT64_BITS))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags64_EnumFindFirstEnumeratedFlag()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags64 sfTestFlags((uint64ou)(OU_INT64_MIN + 1));

		unsigned int uiFirstResult = sfTestFlags.EnumFindFirstEnumeratedFlag(1, OU_UINT64_BITS);
		if (uiFirstResult != 0)
		{
			break;
		}

		unsigned int uiSecondResult = sfTestFlags.EnumFindFirstEnumeratedFlag(2, OU_UINT64_BITS - 1);
		if (uiSecondResult != OU_UINT64_BITS - 2)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags64_EnumAllSignalEnumeratedFlags()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags64 sfTestFlags;
		
		sfTestFlags.EnumAllSignalEnumeratedFlags(1, 1);

		if (sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}

		sfTestFlags.EnumAllSignalEnumeratedFlags(4, OU_UINT64_BITS - 2);

		if (sfTestFlags.QueryFlagsAllValues() != (uint64ou)(OU_UINT64_MAX ^ 2))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags64_EnumAllDropEnumeratedFlags()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags64 sfTestFlags(OU_UINT64_MAX);
		
		sfTestFlags.EnumAllDropEnumeratedFlags(1, 1);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint64ou)(OU_UINT64_MAX ^ 1))
		{
			break;
		}
		
		sfTestFlags.EnumAllDropEnumeratedFlags(4, OU_UINT64_BITS - 2);
		
		if (sfTestFlags.QueryFlagsAllValues() != 2)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags64_EnumAllQueryEnumeratedFlags()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags64 sfTestFlags((uint64ou)(OU_INT64_MIN + 1));
		
		uint64ou uiFirstResult = sfTestFlags.EnumAllQueryEnumeratedFlags(1, OU_UINT64_BITS);
		if (uiFirstResult != (uint64ou)(OU_INT64_MIN + 1))
		{
			break;
		}
		
		uint64ou uiSecondResult = sfTestFlags.EnumAllQueryEnumeratedFlags(2, OU_UINT64_BITS - 1);
		if (uiSecondResult != (uint64ou)(OU_INT64_MIN))
		{
			break;
		}
		
		uint64ou uiThirdResult = sfTestFlags.EnumAllQueryEnumeratedFlags(2, OU_UINT64_BITS - 2);
		if (uiThirdResult != 0)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags64_EnumAnyGetEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags64 sfTestFlags((uint64ou)(OU_INT64_MIN + 1));
		
		bool bFirstResult = sfTestFlags.EnumAnyGetEnumeratedFlagValue(1, OU_UINT64_BITS);
		if (!bFirstResult)
		{
			break;
		}
		
		bool bSecondResult = sfTestFlags.EnumAnyGetEnumeratedFlagValue(2, OU_UINT64_BITS - 1);
		if (!bSecondResult)
		{
			break;
		}
		
		bool bThirdResult = sfTestFlags.EnumAnyGetEnumeratedFlagValue(2, OU_UINT64_BITS - 2);
		if (bThirdResult)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags64_StoreFlagsEnumeratedValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags64 sfTestFlags;

		sfTestFlags.StoreFlagsEnumeratedValue(0x03, 1, 2);

		if (sfTestFlags.QueryFlagsAllValues() != (uint64ou)(2 << 1))
		{
			break;
		}
	
		sfTestFlags.StoreFlagsEnumeratedValue(0x03, OU_UINT64_BITS - 2, 3);
		
		if (sfTestFlags.QueryFlagsAllValues() != ((uint64ou)(2 << 1) | (uint64ou)(OU_INT64_MIN | (OU_INT64_MIN >> 1))))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags64_RetrieveFlagsEnumeratedValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags64 sfTestFlags((uint64ou)(OU_INT64_MIN + 1));
		
		unsigned int uiFirstResult = sfTestFlags.RetrieveFlagsEnumeratedValue(0x3, 1);
		if (uiFirstResult != 0)
		{
			break;
		}
		
		unsigned int uiSecondResult = sfTestFlags.RetrieveFlagsEnumeratedValue(0x3, OU_UINT64_BITS - 2);
		if (uiSecondResult != 2)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}


enum EOUSIMPLEFLAGSFEATURE64
{
	OSF64__MIN,
		
	OSF64_CONSTRUCTORS = OSF64__MIN,
	OSF64_ASSIGNFLAGSALLVALUES,
	OSF64_QUERYFLAGSALLVALUES,
	OSF64_SETFLAGSMASKVALUE,
	OSF64_SIGNALFLAGSMASKVALUE,
	OSF64_DROPFLAGSMASKVALUE,
	OSF64_TOGGLESINGLEFLAGVALUE,
	OSF64_MODIFYSINGLEFLAGVALUE,
	OSF64_ASSIGNFLAGSBYMASK,
	OSF64_ALTERFLAGSBYMASK,
	OSF64_GETFLAGSMASKVALUE,
	OSF64_QUERYFLAGSBYMASK,
	OSF64_ONLYSIGNALSINGLEFLAGOUTOFMASK,
	OSF64_ENUMSETENUMERATEDFLAGVALUE,
	OSF64_ENUMSIGNALENUMERATEDFLAGVALUE,
	OSF64_ENUMDROPENUMERATEDFLAGVALUE,
	OSF64_ENUMTOGGLEENUMERATEDFLAGVALUE,
	OSF64_ENUMMODIFYENUMERATEDFLAGVALUE,
	OSF64_ENUMSIGNALFIRSTENUMERATEDFLAGVALUE,
	OSF64_ENUMSIGNALLASTENUMERATEDFLAGVALUE,
	OSF64_ENUMGETENUMERATEDFLAGVALUE,
	OSF64_ENUMFINDFIRSTENUMERATEDFLAG,
	OSF64_ENUMALLSIGNALENUMERATEDFLAGS,
	OSF64_ENUMALLDROPENUMERATEDFLAGS,
	OSF64_ENUMALLQUERYENUMERATEDFLAGS,
	OSF64_ENUMANYGETENUMERATEDFLAGVALUE,
	OSF64_STOREFLAGSENUMERATEDVALUE,
	OSF64_RETRIEVEFLAGSENUMERATEDVALUE,

	OSF64__MAX,
};

template<>
CFeatureTestProcedure const CEnumUnsortedElementArray<EOUSIMPLEFLAGSFEATURE64, OSF64__MAX, CFeatureTestProcedure>::m_aetElementArray[] =
{
	&TestSimpleFlags64_Constructors, // OSF64_CONSTRUCTORS
	&TestSimpleFlags64_AssignFlagsAllValues, // OSF64_ASSIGNFLAGSALLVALUES,
	&TestSimpleFlags64_QueryFlagsAllValues, // OSF64_QUERYFLAGSALLVALUES,
	&TestSimpleFlags64_SetFlagsMaskValue, // OSF64_SETFLAGSMASKVALUE,
	&TestSimpleFlags64_SignalFlagsMaskValue, // OSF64_SIGNALFLAGSMASKVALUE,
	&TestSimpleFlags64_DropFlagsMaskValue, // OSF64_DROPFLAGSMASKVALUE,
	&TestSimpleFlags64_ToggleSingleFlagValue, // OSF64_TOGGLESINGLEFLAGVALUE,
	&TestSimpleFlags64_ModifySingleFlagValue, // OSF64_MODIFYSINGLEFLAGVALUE,
	&TestSimpleFlags64_AssignFlagsByMask, // OSF64_ASSIGNFLAGSBYMASK,
	&TestSimpleFlags64_AlterFlagsByMask, // OSF64_ALTERFLAGSBYMASK,
	&TestSimpleFlags64_GetFlagsMaskValue, // OSF64_GETFLAGSMASKVALUE,
	&TestSimpleFlags64_QueryFlagsByMask, // OSF64_QUERYFLAGSBYMASK,
	&TestSimpleFlags64_OnlySignalSingleFlagOutOfMask, // OSF64_ONLYSIGNALSINGLEFLAGOUTOFMASK,
	&TestSimpleFlags64_EnumSetEnumeratedFlagValue, // OSF64_ENUMSETENUMERATEDFLAGVALUE,
	&TestSimpleFlags64_EnumSignalEnumeratedFlagValue, // OSF64_ENUMSIGNALENUMERATEDFLAGVALUE,
	&TestSimpleFlags64_EnumDropEnumeratedFlagValue, // OSF64_ENUMDROPENUMERATEDFLAGVALUE,
	&TestSimpleFlags64_EnumToggleEnumeratedFlagValue, // OSF64_ENUMTOGGLEENUMERATEDFLAGVALUE,
	&TestSimpleFlags64_EnumModifyEnumeratedFlagValue, // OSF64_ENUMMODIFYENUMERATEDFLAGVALUE,
	&TestSimpleFlags64_EnumSignalFirstEnumeratedFlagValue, // OSF64_ENUMSIGNALFIRSTENUMERATEDFLAGVALUE,
	&TestSimpleFlags64_EnumSignalLastEnumeratedFlagValue, // OSF64_ENUMSIGNALLASTENUMERATEDFLAGVALUE,
	&TestSimpleFlags64_EnumGetEnumeratedFlagValue, // OSF64_ENUMGETENUMERATEDFLAGVALUE,
	&TestSimpleFlags64_EnumFindFirstEnumeratedFlag, // OSF64_ENUMFINDFIRSTENUMERATEDFLAG,
	&TestSimpleFlags64_EnumAllSignalEnumeratedFlags, // OSF64_ENUMALLSIGNALENUMERATEDFLAGS,
	&TestSimpleFlags64_EnumAllDropEnumeratedFlags, // OSF64_ENUMALLDROPENUMERATEDFLAGS,
	&TestSimpleFlags64_EnumAllQueryEnumeratedFlags, // OSF64_ENUMALLQUERYENUMERATEDFLAGS,
	&TestSimpleFlags64_EnumAnyGetEnumeratedFlagValue, // OSF64_ENUMANYGETENUMERATEDFLAGVALUE,
	&TestSimpleFlags64_StoreFlagsEnumeratedValue, // OSF64_STOREFLAGSENUMERATEDVALUE,
	&TestSimpleFlags64_RetrieveFlagsEnumeratedValue, // OSF64_RETRIEVEFLAGSENUMERATEDVALUE,
};
static const CEnumUnsortedElementArray<EOUSIMPLEFLAGSFEATURE64, OSF64__MAX, CFeatureTestProcedure> g_afnSimpleFlags64FeatureTestProcedures;

template<>
const char *const CEnumUnsortedElementArray<EOUSIMPLEFLAGSFEATURE64, OSF64__MAX, const char *>::m_aetElementArray[] =
{
	"Constructors", // OSF64_CONSTRUCTORS
	"AssignFlagsAllValues", // OSF64_ASSIGNFLAGSALLVALUES,
	"QueryFlagsAllValues", // OSF64_QUERYFLAGSALLVALUES,
	"SetFlagsMaskValue", // OSF64_SETFLAGSMASKVALUE,
	"SignalFlagsMaskValue", // OSF64_SIGNALFLAGSMASKVALUE,
	"DropFlagsMaskValue", // OSF64_DROPFLAGSMASKVALUE,
	"ToggleSingleFlagValue", // OSF64_TOGGLESINGLEFLAGVALUE,
	"ModifySingleFlagValue", // OSF64_MODIFYSINGLEFLAGVALUE,
	"AssignFlagsByMask", // OSF64_ASSIGNFLAGSBYMASK,
	"AlterFlagsByMask", // OSF64_ALTERFLAGSBYMASK,
	"GetFlagsMaskValue", // OSF64_GETFLAGSMASKVALUE,
	"QueryFlagsByMask", // OSF64_QUERYFLAGSBYMASK,
	"OnlySignalSingleFlagOutOfMask", // OSF64_ONLYSIGNALSINGLEFLAGOUTOFMASK,
	"EnumSetEnumeratedFlagValue", // OSF64_ENUMSETENUMERATEDFLAGVALUE,
	"EnumSignalEnumeratedFlagValue", // OSF64_ENUMSIGNALENUMERATEDFLAGVALUE,
	"EnumDropEnumeratedFlagValue", // OSF64_ENUMDROPENUMERATEDFLAGVALUE,
	"EnumToggleEnumeratedFlagValue", // OSF64_ENUMTOGGLEENUMERATEDFLAGVALUE,
	"EnumModifyEnumeratedFlagValue", // OSF64_ENUMMODIFYENUMERATEDFLAGVALUE,
	"EnumSignalFirstEnumeratedFlagValue", // OSF64_ENUMSIGNALFIRSTENUMERATEDFLAGVALUE,
	"EnumSignalLastEnumeratedFlagValue", // OSF64_ENUMSIGNALLASTENUMERATEDFLAGVALUE,
	"EnumGetEnumeratedFlagValue", // OSF64_ENUMGETENUMERATEDFLAGVALUE,
	"EnumFindFirstEnumeratedFlag", // OSF64_ENUMFINDFIRSTENUMERATEDFLAG,
	"EnumAllSignalEnumeratedFlags", // OSF64_ENUMALLSIGNALENUMERATEDFLAGS,
	"EnumAllDropEnumeratedFlags", // OSF64_ENUMALLDROPENUMERATEDFLAGS,
	"EnumAllQueryEnumeratedFlags", // OSF64_ENUMALLQUERYENUMERATEDFLAGS,
	"EnumAnyGetEnumeratedFlagValue", // OSF64_ENUMANYGETENUMERATEDFLAGVALUE,
	"StoreFlagsEnumeratedValue", // OSF64_STOREFLAGSENUMERATEDVALUE,
	"RetrieveFlagsEnumeratedValue", // OSF64_RETRIEVEFLAGSENUMERATEDVALUE,
};
static const CEnumUnsortedElementArray<EOUSIMPLEFLAGSFEATURE64, OSF64__MAX, const char *> g_aszSimpleFlags64FeatureTestNames;


bool TestSimpleFlags64(unsigned int &nOutSuccessCount, unsigned int &nOutTestCount)
{
	return TestSubsystem(nOutSuccessCount, nOutTestCount, OSF64__MAX, g_aszSimpleFlags64FeatureTestNames.GetStoragePointer(), g_afnSimpleFlags64FeatureTestProcedures.GetStoragePointer());
}


//////////////////////////////////////////////////////////////////////////

typedef CSimpleFlags CSimpleFlags32;

const uint32ou g_uiTestValue32 = (uint32ou)0xA5A5A5A5;
const uint32ou g_uiTestMask32 = (uint32ou)0xC6C6C6C6;
const uint32ou g_uiTestBit32 = (uint32ou)OU_INT32_MIN;
const uint32ou g_uiTestAnotherBit32 = (uint32ou)((uint32ou)OU_INT32_MIN >> 1);


bool TestSimpleFlags32_Constructors()
{
	bool bResult = false;
	
	do
	{
		if (sizeof(CSimpleFlags32::value_type) != sizeof(uint32ou) || sizeof(CSimpleFlags32) != sizeof(uint32ou))
		{
			break;
		}

		CSimpleFlags32 sfEmptyFlags;

		if (sfEmptyFlags.QueryFlagsAllValues())
		{
			break;
		}

		CSimpleFlags32 sfFullFlags(OU_UINT32_MAX);
		
		if (sfFullFlags.QueryFlagsAllValues() != OU_UINT32_MAX)
		{
			break;
		}

		CSimpleFlags32 sfCopyOfFullFlags(sfFullFlags);

		if (sfCopyOfFullFlags.QueryFlagsAllValues() != OU_UINT32_MAX)
		{
			break;
		}
	
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags32_AssignFlagsAllValues()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags32 sfTestFlags;

		sfTestFlags.AssignFlagsAllValues(OU_UINT32_MAX);

		if (sfTestFlags.QueryFlagsAllValues() != OU_UINT32_MAX)
		{
			break;
		}

		sfTestFlags.AssignFlagsAllValues(0);

		if (sfTestFlags.QueryFlagsAllValues() != 0)
		{
			break;
		}
	
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags32_QueryFlagsAllValues()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags32 sfTestFlags(g_uiTestValue32);
		
		if (sfTestFlags.QueryFlagsAllValues() != g_uiTestValue32)
		{
			break;
		}

		// Double check to be sure ;-)
		if (sfTestFlags.QueryFlagsAllValues() != g_uiTestValue32)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags32_SetFlagsMaskValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags32 sfTestFlags(g_uiTestValue32);

		sfTestFlags.SetFlagsMaskValue(g_uiTestMask32, true);

		if (sfTestFlags.QueryFlagsAllValues() != (uint32ou)(g_uiTestValue32 | g_uiTestMask32))
		{
			break;
		}

		sfTestFlags.SetFlagsMaskValue(g_uiTestValue32, false);

		if (sfTestFlags.QueryFlagsAllValues() != (uint32ou)(~g_uiTestValue32 & g_uiTestMask32))
		{
			break;
		}
		

		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags32_SignalFlagsMaskValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags32 sfTestFlags(g_uiTestValue32);
		
		sfTestFlags.SignalFlagsMaskValue(g_uiTestMask32);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint32ou)(g_uiTestValue32 | g_uiTestMask32))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags32_DropFlagsMaskValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags32 sfTestFlags(g_uiTestValue32);
		
		sfTestFlags.DropFlagsMaskValue(g_uiTestMask32);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint32ou)(g_uiTestValue32 & ~g_uiTestMask32))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags32_ToggleSingleFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags32 sfTestFlags(g_uiTestValue32);
		
		bool bPreviousValue = sfTestFlags.ToggleSingleFlagValue(g_uiTestBit32);
		
		if (bPreviousValue != ((g_uiTestValue32 & g_uiTestBit32) != 0) || sfTestFlags.QueryFlagsAllValues() != (uint32ou)(g_uiTestValue32 ^ g_uiTestBit32))
		{
			break;
		}

		bool bAnotherPreviousValue = sfTestFlags.ToggleSingleFlagValue(g_uiTestBit32);
		
		if (bAnotherPreviousValue == bPreviousValue || sfTestFlags.QueryFlagsAllValues() != g_uiTestValue32)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags32_ModifySingleFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags32 sfTestFlags(g_uiTestValue32);
		
		bool bFirstModification = sfTestFlags.ModifySingleFlagValue(g_uiTestBit32, true);
		
		if (bFirstModification != ((g_uiTestValue32 & g_uiTestBit32) != g_uiTestBit32) || sfTestFlags.QueryFlagsAllValues() != (uint32ou)(g_uiTestValue32 | g_uiTestBit32))
		{
			break;
		}
		
		bool bAnotherModification = sfTestFlags.ModifySingleFlagValue(g_uiTestBit32, bFirstModification);
		
		if (bAnotherModification == bFirstModification || sfTestFlags.QueryFlagsAllValues() != (bFirstModification ? (uint32ou)(g_uiTestValue32 | g_uiTestBit32) : (uint32ou)(g_uiTestValue32 & ~g_uiTestBit32)))
		{
			break;
		}

		bool bYetAnotherModification = sfTestFlags.ModifySingleFlagValue(g_uiTestBit32, bAnotherModification);

		if (bYetAnotherModification != bAnotherModification || sfTestFlags.QueryFlagsAllValues() != (bAnotherModification ? (uint32ou)(g_uiTestValue32 | g_uiTestBit32) : (uint32ou)(g_uiTestValue32 & ~g_uiTestBit32)))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags32_AssignFlagsByMask()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags32 sfTestFlags(g_uiTestValue32);

		uint32ou uiPreviousFlags = sfTestFlags.AssignFlagsByMask(g_uiTestMask32, g_uiTestMask32);

		const uint32ou uiNewFlags = (g_uiTestValue32 & ~g_uiTestMask32) | g_uiTestMask32;

		if (uiPreviousFlags != g_uiTestValue32 || sfTestFlags.QueryFlagsAllValues() != uiNewFlags)
		{
			break;
		}

		uint32ou uiAnotherPreviousFlags = sfTestFlags.AssignFlagsByMask(g_uiTestValue32, 0);

		const uint32ou uiAnotherNewFlags = uiNewFlags & ~g_uiTestValue32;

		if (uiAnotherPreviousFlags != uiNewFlags || sfTestFlags.QueryFlagsAllValues() != uiAnotherNewFlags)
		{
			break;
		}
	
		uint32ou uiYetAnotherPreviousFlags = sfTestFlags.AssignFlagsByMask(g_uiTestMask32, g_uiTestMask32 & g_uiTestValue32);
		OU_ASSERT((g_uiTestMask32 & g_uiTestValue32) != 0); // Test degeneration
		
		const uint32ou uiYetAnotherNewFlags = (uiAnotherNewFlags & ~g_uiTestMask32) | (g_uiTestMask32 & g_uiTestValue32);
		OU_ASSERT(uiYetAnotherNewFlags != OU_UINT32_MAX); // Test degeneration
		
		if (uiYetAnotherPreviousFlags != uiAnotherNewFlags || sfTestFlags.QueryFlagsAllValues() != uiYetAnotherNewFlags)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags32_AlterFlagsByMask()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags32 sfTestFlags(g_uiTestValue32);
		
		bool bWasModification = sfTestFlags.AlterFlagsByMask(g_uiTestMask32, g_uiTestMask32);
		
		const uint32ou uiNewFlags = (g_uiTestValue32 & ~g_uiTestMask32) | g_uiTestMask32;
		
		if (bWasModification != ((g_uiTestValue32 & g_uiTestMask32) != g_uiTestMask32) || sfTestFlags.QueryFlagsAllValues() != uiNewFlags)
		{
			break;
		}
		
		bool bWasAnotherModification = sfTestFlags.AlterFlagsByMask(g_uiTestValue32, 0);
		
		const uint32ou uiAnotherNewFlags = uiNewFlags & ~g_uiTestValue32;
		
		if (bWasAnotherModification != ((uiNewFlags & g_uiTestValue32) != 0) || sfTestFlags.QueryFlagsAllValues() != uiAnotherNewFlags)
		{
			break;
		}
		
		bool bWasAnotherModificationRepeated = sfTestFlags.AlterFlagsByMask(g_uiTestValue32, 0);
		
		if (bWasAnotherModificationRepeated || sfTestFlags.QueryFlagsAllValues() != uiAnotherNewFlags)
		{
			break;
		}
		
		bool bWasYetAnotherModification = sfTestFlags.AlterFlagsByMask(g_uiTestMask32, g_uiTestMask32 & g_uiTestValue32);
		OU_ASSERT((g_uiTestMask32 & g_uiTestValue32) != 0); // Test degeneration
		
		const uint32ou uiYetAnotherNewFlags = (uiAnotherNewFlags & ~g_uiTestMask32) | (g_uiTestMask32 & g_uiTestValue32);
		OU_ASSERT(uiYetAnotherNewFlags != OU_UINT32_MAX); // Test degeneration
		
		if (bWasYetAnotherModification != ((uiAnotherNewFlags & g_uiTestMask32) != (g_uiTestMask32 & g_uiTestValue32)) || sfTestFlags.QueryFlagsAllValues() != uiYetAnotherNewFlags)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags32_GetFlagsMaskValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags32 sfTestFlags(g_uiTestValue32);
		
		if (sfTestFlags.GetFlagsMaskValue(g_uiTestMask32) != ((g_uiTestValue32 & g_uiTestMask32) != 0))
		{
			break;
		}
		
		if (sfTestFlags.GetFlagsMaskValue(~g_uiTestValue32))
		{
			break;
		}
		
		if (!sfTestFlags.GetFlagsMaskValue(OU_UINT32_MAX))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags32_QueryFlagsByMask()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags32 sfTestFlags(g_uiTestValue32);
		
		if (sfTestFlags.QueryFlagsByMask(g_uiTestMask32) != (uint32ou)(g_uiTestValue32 & g_uiTestMask32))
		{
			break;
		}
		
		if (sfTestFlags.QueryFlagsByMask(0))
		{
			break;
		}
		
		if (sfTestFlags.QueryFlagsByMask(OU_UINT32_MAX) != g_uiTestValue32)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags32_OnlySignalSingleFlagOutOfMask()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags32 sfTestFlags(g_uiTestValue32);
		OU_ASSERT(g_uiTestValue32 != 0); // Test degeneration
		
		if (sfTestFlags.OnlySignalSingleFlagOutOfMask(OU_UINT32_MAX, g_uiTestBit32))
		{
			break;
		}

		if (sfTestFlags.QueryFlagsAllValues() != g_uiTestValue32)
		{
			break;
		}
		
		sfTestFlags.AssignFlagsAllValues(0);

		if (!sfTestFlags.OnlySignalSingleFlagOutOfMask(g_uiTestBit32, g_uiTestBit32))
		{
			break;
		}
		
		if (sfTestFlags.QueryFlagsAllValues() != g_uiTestBit32)
		{
			break;
		}
		
		if (sfTestFlags.OnlySignalSingleFlagOutOfMask(g_uiTestBit32, g_uiTestBit32))
		{
			break;
		}
		
		if (sfTestFlags.QueryFlagsAllValues() != g_uiTestBit32)
		{
			break;
		}
		
		if (sfTestFlags.OnlySignalSingleFlagOutOfMask(OU_UINT32_MAX, g_uiTestAnotherBit32))
		{
			break;
		}
		
		if (sfTestFlags.QueryFlagsAllValues() != g_uiTestBit32)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags32_EnumSetEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags32 sfTestFlags;
		
		sfTestFlags.EnumSetEnumeratedFlagValue(1, 0, OU_UINT32_BITS, true);

		if (sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
	
		sfTestFlags.EnumSetEnumeratedFlagValue(1, OU_UINT32_BITS - 1, OU_UINT32_BITS, true);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint32ou)(OU_INT32_MIN + 1))
		{
			break;
		}
		
		sfTestFlags.EnumSetEnumeratedFlagValue(1, 0, OU_UINT32_BITS, false);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint32ou)OU_INT32_MIN)
		{
			break;
		}
		
		sfTestFlags.EnumSetEnumeratedFlagValue(1, OU_UINT32_BITS - 1, OU_UINT32_BITS, false);
		
		if (sfTestFlags.QueryFlagsAllValues() != 0)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags32_EnumSignalEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags32 sfTestFlags;
		
		sfTestFlags.EnumSignalEnumeratedFlagValue(1, 0, OU_UINT32_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		sfTestFlags.EnumSignalEnumeratedFlagValue(1, OU_UINT32_BITS - 1, OU_UINT32_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint32ou)(OU_INT32_MIN + 1))
		{
			break;
		}
		
		sfTestFlags.EnumSignalEnumeratedFlagValue(1, 0, OU_UINT32_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint32ou)(OU_INT32_MIN + 1))
		{
			break;
		}
		
		sfTestFlags.EnumSignalEnumeratedFlagValue(1, OU_UINT32_BITS - 1, OU_UINT32_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint32ou)(OU_INT32_MIN + 1))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags32_EnumDropEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags32 sfTestFlags(OU_UINT32_MAX);
		
		sfTestFlags.EnumDropEnumeratedFlagValue(1, 0, OU_UINT32_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != (OU_UINT32_MAX ^ 1))
		{
			break;
		}
		
		sfTestFlags.EnumDropEnumeratedFlagValue(1, OU_UINT32_BITS - 1, OU_UINT32_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint32ou)~(OU_INT32_MIN + 1))
		{
			break;
		}
		
		sfTestFlags.EnumDropEnumeratedFlagValue(1, 0, OU_UINT32_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint32ou)~(OU_INT32_MIN + 1))
		{
			break;
		}
		
		sfTestFlags.EnumDropEnumeratedFlagValue(1, OU_UINT32_BITS - 1, OU_UINT32_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint32ou)~(OU_INT32_MIN + 1))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags32_EnumToggleEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags32 sfTestFlags;
		
		bool bToggleFirstResult = sfTestFlags.EnumToggleEnumeratedFlagValue(1, 0, OU_UINT32_BITS);
		
		if (bToggleFirstResult || sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		bool bToggleSecondResult = sfTestFlags.EnumToggleEnumeratedFlagValue(1, OU_UINT32_BITS - 1, OU_UINT32_BITS);
		
		if (bToggleSecondResult || sfTestFlags.QueryFlagsAllValues() != (uint32ou)(OU_INT32_MIN + 1))
		{
			break;
		}
		
		bool bToggleThirdResult = sfTestFlags.EnumToggleEnumeratedFlagValue(1, 0, OU_UINT32_BITS);
		
		if (!bToggleThirdResult || sfTestFlags.QueryFlagsAllValues() != (uint32ou)OU_INT32_MIN)
		{
			break;
		}
		
		bool bToggleFourthResult = sfTestFlags.EnumToggleEnumeratedFlagValue(1, OU_UINT32_BITS - 1, OU_UINT32_BITS);
		
		if (!bToggleFourthResult || sfTestFlags.QueryFlagsAllValues() != 0)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags32_EnumModifyEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags32 sfTestFlags;
		
		bool bModifyFirstResult = sfTestFlags.EnumModifyEnumeratedFlagValue(1, 0, OU_UINT32_BITS, true);
		
		if (!bModifyFirstResult || sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		bool bModifySecondResult = sfTestFlags.EnumModifyEnumeratedFlagValue(1, OU_UINT32_BITS - 1, OU_UINT32_BITS, true);
		
		if (!bModifySecondResult || sfTestFlags.QueryFlagsAllValues() != (uint32ou)(OU_INT32_MIN + 1))
		{
			break;
		}
		
		bool bModifyThirdResult = sfTestFlags.EnumModifyEnumeratedFlagValue(1, 0, OU_UINT32_BITS, true);
		
		if (bModifyThirdResult || sfTestFlags.QueryFlagsAllValues() != (uint32ou)(OU_INT32_MIN + 1))
		{
			break;
		}
		
		bool bModifyFourthResult = sfTestFlags.EnumModifyEnumeratedFlagValue(1, OU_UINT32_BITS - 1, OU_UINT32_BITS, true);
		
		if (bModifyFourthResult || sfTestFlags.QueryFlagsAllValues() != (uint32ou)(OU_INT32_MIN + 1))
		{
			break;
		}
		
		bool bModifyFifthResult = sfTestFlags.EnumModifyEnumeratedFlagValue(1, 0, OU_UINT32_BITS, false);
		
		if (!bModifyFifthResult || sfTestFlags.QueryFlagsAllValues() != (uint32ou)OU_INT32_MIN)
		{
			break;
		}
		
		bool bModifySixthResult = sfTestFlags.EnumModifyEnumeratedFlagValue(1, OU_UINT32_BITS - 1, OU_UINT32_BITS, false);
		
		if (!bModifySixthResult || sfTestFlags.QueryFlagsAllValues() != 0)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags32_EnumSignalFirstEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags32 sfTestFlags;

		bool bFirstResult = sfTestFlags.EnumSignalFirstEnumeratedFlagValue(1, 0, OU_UINT32_BITS);

		if (!bFirstResult || sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		bool bSecondResult = sfTestFlags.EnumSignalFirstEnumeratedFlagValue(1, 0, OU_UINT32_BITS);
		
		if (bSecondResult || sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		bool bThirdResult = sfTestFlags.EnumSignalFirstEnumeratedFlagValue(2, 0, OU_UINT32_BITS - 1);
		
		if (!bThirdResult || sfTestFlags.QueryFlagsAllValues() != 3)
		{
			break;
		}
		
		bool bFourthResult = sfTestFlags.EnumSignalFirstEnumeratedFlagValue(1, OU_UINT32_BITS - 1, OU_UINT32_BITS);
		
		if (bFourthResult || sfTestFlags.QueryFlagsAllValues() != (uint32ou)(OU_INT32_MIN + 3))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags32_EnumSignalLastEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags32 sfTestFlags;
		
		bool bFirstResult = sfTestFlags.EnumSignalLastEnumeratedFlagValue(1, 0, 1);
		
		if (!bFirstResult || sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		bool bSecondResult = sfTestFlags.EnumSignalLastEnumeratedFlagValue(1, 0, 1);
		
		if (bSecondResult || sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		bool bThirdResult = sfTestFlags.EnumSignalLastEnumeratedFlagValue(1, 1, 2);
		
		if (!bThirdResult || sfTestFlags.QueryFlagsAllValues() != 3)
		{
			break;
		}
		
		bool bFourthResult = sfTestFlags.EnumSignalLastEnumeratedFlagValue(1, OU_UINT32_BITS - 1, OU_UINT32_BITS);
		
		if (bFourthResult || sfTestFlags.QueryFlagsAllValues() != (uint32ou)(OU_INT32_MIN + 3))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags32_EnumGetEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags32 sfTestFlags((uint32ou)(OU_INT32_MIN + 1));
		
		if (!sfTestFlags.EnumGetEnumeratedFlagValue(1, 0, OU_UINT32_BITS))
		{
			break;
		}
	
		if (sfTestFlags.EnumGetEnumeratedFlagValue(2, 0, OU_UINT32_BITS - 1))
		{
			break;
		}
		
		if (sfTestFlags.EnumGetEnumeratedFlagValue(1, 1, OU_UINT32_BITS - 1))
		{
			break;
		}
		
		if (!sfTestFlags.EnumGetEnumeratedFlagValue(1, OU_UINT32_BITS - 1, OU_UINT32_BITS))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags32_EnumFindFirstEnumeratedFlag()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags32 sfTestFlags((uint32ou)(OU_INT32_MIN + 1));

		unsigned int uiFirstResult = sfTestFlags.EnumFindFirstEnumeratedFlag(1, OU_UINT32_BITS);
		if (uiFirstResult != 0)
		{
			break;
		}

		unsigned int uiSecondResult = sfTestFlags.EnumFindFirstEnumeratedFlag(2, OU_UINT32_BITS - 1);
		if (uiSecondResult != OU_UINT32_BITS - 2)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags32_EnumAllSignalEnumeratedFlags()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags32 sfTestFlags;
		
		sfTestFlags.EnumAllSignalEnumeratedFlags(1, 1);

		if (sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}

		sfTestFlags.EnumAllSignalEnumeratedFlags(4, OU_UINT32_BITS - 2);

		if (sfTestFlags.QueryFlagsAllValues() != (uint32ou)(OU_UINT32_MAX ^ 2))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags32_EnumAllDropEnumeratedFlags()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags32 sfTestFlags(OU_UINT32_MAX);
		
		sfTestFlags.EnumAllDropEnumeratedFlags(1, 1);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint32ou)(OU_UINT32_MAX ^ 1))
		{
			break;
		}
		
		sfTestFlags.EnumAllDropEnumeratedFlags(4, OU_UINT32_BITS - 2);
		
		if (sfTestFlags.QueryFlagsAllValues() != 2)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags32_EnumAllQueryEnumeratedFlags()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags32 sfTestFlags((uint32ou)(OU_INT32_MIN + 1));
		
		uint32ou uiFirstResult = sfTestFlags.EnumAllQueryEnumeratedFlags(1, OU_UINT32_BITS);
		if (uiFirstResult != (uint32ou)(OU_INT32_MIN + 1))
		{
			break;
		}
		
		uint32ou uiSecondResult = sfTestFlags.EnumAllQueryEnumeratedFlags(2, OU_UINT32_BITS - 1);
		if (uiSecondResult != (uint32ou)(OU_INT32_MIN))
		{
			break;
		}
		
		uint32ou uiThirdResult = sfTestFlags.EnumAllQueryEnumeratedFlags(2, OU_UINT32_BITS - 2);
		if (uiThirdResult != 0)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags32_EnumAnyGetEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags32 sfTestFlags((uint32ou)(OU_INT32_MIN + 1));
		
		bool bFirstResult = sfTestFlags.EnumAnyGetEnumeratedFlagValue(1, OU_UINT32_BITS);
		if (!bFirstResult)
		{
			break;
		}
		
		bool bSecondResult = sfTestFlags.EnumAnyGetEnumeratedFlagValue(2, OU_UINT32_BITS - 1);
		if (!bSecondResult)
		{
			break;
		}
		
		bool bThirdResult = sfTestFlags.EnumAnyGetEnumeratedFlagValue(2, OU_UINT32_BITS - 2);
		if (bThirdResult)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags32_StoreFlagsEnumeratedValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags32 sfTestFlags;

		sfTestFlags.StoreFlagsEnumeratedValue(0x03, 1, 2);

		if (sfTestFlags.QueryFlagsAllValues() != (uint32ou)(2 << 1))
		{
			break;
		}
	
		sfTestFlags.StoreFlagsEnumeratedValue(0x03, OU_UINT32_BITS - 2, 3);
		
		if (sfTestFlags.QueryFlagsAllValues() != ((uint32ou)(2 << 1) | (uint32ou)(OU_INT32_MIN | (OU_INT32_MIN >> 1))))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags32_RetrieveFlagsEnumeratedValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags32 sfTestFlags((uint32ou)(OU_INT32_MIN + 1));
		
		unsigned int uiFirstResult = sfTestFlags.RetrieveFlagsEnumeratedValue(0x3, 1);
		if (uiFirstResult != 0)
		{
			break;
		}
		
		unsigned int uiSecondResult = sfTestFlags.RetrieveFlagsEnumeratedValue(0x3, OU_UINT32_BITS - 2);
		if (uiSecondResult != 2)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}


enum EOUSIMPLEFLAGSFEATURE32
{
	OSF32__MIN,
		
	OSF32_CONSTRUCTORS = OSF32__MIN,
	OSF32_ASSIGNFLAGSALLVALUES,
	OSF32_QUERYFLAGSALLVALUES,
	OSF32_SETFLAGSMASKVALUE,
	OSF32_SIGNALFLAGSMASKVALUE,
	OSF32_DROPFLAGSMASKVALUE,
	OSF32_TOGGLESINGLEFLAGVALUE,
	OSF32_MODIFYSINGLEFLAGVALUE,
	OSF32_ASSIGNFLAGSBYMASK,
	OSF32_ALTERFLAGSBYMASK,
	OSF32_GETFLAGSMASKVALUE,
	OSF32_QUERYFLAGSBYMASK,
	OSF32_ONLYSIGNALSINGLEFLAGOUTOFMASK,
	OSF32_ENUMSETENUMERATEDFLAGVALUE,
	OSF32_ENUMSIGNALENUMERATEDFLAGVALUE,
	OSF32_ENUMDROPENUMERATEDFLAGVALUE,
	OSF32_ENUMTOGGLEENUMERATEDFLAGVALUE,
	OSF32_ENUMMODIFYENUMERATEDFLAGVALUE,
	OSF32_ENUMSIGNALFIRSTENUMERATEDFLAGVALUE,
	OSF32_ENUMSIGNALLASTENUMERATEDFLAGVALUE,
	OSF32_ENUMGETENUMERATEDFLAGVALUE,
	OSF32_ENUMFINDFIRSTENUMERATEDFLAG,
	OSF32_ENUMALLSIGNALENUMERATEDFLAGS,
	OSF32_ENUMALLDROPENUMERATEDFLAGS,
	OSF32_ENUMALLQUERYENUMERATEDFLAGS,
	OSF32_ENUMANYGETENUMERATEDFLAGVALUE,
	OSF32_STOREFLAGSENUMERATEDVALUE,
	OSF32_RETRIEVEFLAGSENUMERATEDVALUE,

	OSF32__MAX,
};

template<>
CFeatureTestProcedure const CEnumUnsortedElementArray<EOUSIMPLEFLAGSFEATURE32, OSF32__MAX, CFeatureTestProcedure>::m_aetElementArray[] =
{
	&TestSimpleFlags32_Constructors, // OSF32_CONSTRUCTORS
	&TestSimpleFlags32_AssignFlagsAllValues, // OSF32_ASSIGNFLAGSALLVALUES,
	&TestSimpleFlags32_QueryFlagsAllValues, // OSF32_QUERYFLAGSALLVALUES,
	&TestSimpleFlags32_SetFlagsMaskValue, // OSF32_SETFLAGSMASKVALUE,
	&TestSimpleFlags32_SignalFlagsMaskValue, // OSF32_SIGNALFLAGSMASKVALUE,
	&TestSimpleFlags32_DropFlagsMaskValue, // OSF32_DROPFLAGSMASKVALUE,
	&TestSimpleFlags32_ToggleSingleFlagValue, // OSF32_TOGGLESINGLEFLAGVALUE,
	&TestSimpleFlags32_ModifySingleFlagValue, // OSF32_MODIFYSINGLEFLAGVALUE,
	&TestSimpleFlags32_AssignFlagsByMask, // OSF32_ASSIGNFLAGSBYMASK,
	&TestSimpleFlags32_AlterFlagsByMask, // OSF32_ALTERFLAGSBYMASK,
	&TestSimpleFlags32_GetFlagsMaskValue, // OSF32_GETFLAGSMASKVALUE,
	&TestSimpleFlags32_QueryFlagsByMask, // OSF32_QUERYFLAGSBYMASK,
	&TestSimpleFlags32_OnlySignalSingleFlagOutOfMask, // OSF32_ONLYSIGNALSINGLEFLAGOUTOFMASK,
	&TestSimpleFlags32_EnumSetEnumeratedFlagValue, // OSF32_ENUMSETENUMERATEDFLAGVALUE,
	&TestSimpleFlags32_EnumSignalEnumeratedFlagValue, // OSF32_ENUMSIGNALENUMERATEDFLAGVALUE,
	&TestSimpleFlags32_EnumDropEnumeratedFlagValue, // OSF32_ENUMDROPENUMERATEDFLAGVALUE,
	&TestSimpleFlags32_EnumToggleEnumeratedFlagValue, // OSF32_ENUMTOGGLEENUMERATEDFLAGVALUE,
	&TestSimpleFlags32_EnumModifyEnumeratedFlagValue, // OSF32_ENUMMODIFYENUMERATEDFLAGVALUE,
	&TestSimpleFlags32_EnumSignalFirstEnumeratedFlagValue, // OSF32_ENUMSIGNALFIRSTENUMERATEDFLAGVALUE,
	&TestSimpleFlags32_EnumSignalLastEnumeratedFlagValue, // OSF32_ENUMSIGNALLASTENUMERATEDFLAGVALUE,
	&TestSimpleFlags32_EnumGetEnumeratedFlagValue, // OSF32_ENUMGETENUMERATEDFLAGVALUE,
	&TestSimpleFlags32_EnumFindFirstEnumeratedFlag, // OSF32_ENUMFINDFIRSTENUMERATEDFLAG,
	&TestSimpleFlags32_EnumAllSignalEnumeratedFlags, // OSF32_ENUMALLSIGNALENUMERATEDFLAGS,
	&TestSimpleFlags32_EnumAllDropEnumeratedFlags, // OSF32_ENUMALLDROPENUMERATEDFLAGS,
	&TestSimpleFlags32_EnumAllQueryEnumeratedFlags, // OSF32_ENUMALLQUERYENUMERATEDFLAGS,
	&TestSimpleFlags32_EnumAnyGetEnumeratedFlagValue, // OSF32_ENUMANYGETENUMERATEDFLAGVALUE,
	&TestSimpleFlags32_StoreFlagsEnumeratedValue, // OSF32_STOREFLAGSENUMERATEDVALUE,
	&TestSimpleFlags32_RetrieveFlagsEnumeratedValue, // OSF32_RETRIEVEFLAGSENUMERATEDVALUE,
};
static const CEnumUnsortedElementArray<EOUSIMPLEFLAGSFEATURE32, OSF32__MAX, CFeatureTestProcedure> g_afnSimpleFlags32FeatureTestProcedures;

template<>
const char *const CEnumUnsortedElementArray<EOUSIMPLEFLAGSFEATURE32, OSF32__MAX, const char *>::m_aetElementArray[] =
{
	"Constructors", // OSF32_CONSTRUCTORS
	"AssignFlagsAllValues", // OSF32_ASSIGNFLAGSALLVALUES,
	"QueryFlagsAllValues", // OSF32_QUERYFLAGSALLVALUES,
	"SetFlagsMaskValue", // OSF32_SETFLAGSMASKVALUE,
	"SignalFlagsMaskValue", // OSF32_SIGNALFLAGSMASKVALUE,
	"DropFlagsMaskValue", // OSF32_DROPFLAGSMASKVALUE,
	"ToggleSingleFlagValue", // OSF32_TOGGLESINGLEFLAGVALUE,
	"ModifySingleFlagValue", // OSF32_MODIFYSINGLEFLAGVALUE,
	"AssignFlagsByMask", // OSF32_ASSIGNFLAGSBYMASK,
	"AlterFlagsByMask", // OSF32_ALTERFLAGSBYMASK,
	"GetFlagsMaskValue", // OSF32_GETFLAGSMASKVALUE,
	"QueryFlagsByMask", // OSF32_QUERYFLAGSBYMASK,
	"OnlySignalSingleFlagOutOfMask", // OSF32_ONLYSIGNALSINGLEFLAGOUTOFMASK,
	"EnumSetEnumeratedFlagValue", // OSF32_ENUMSETENUMERATEDFLAGVALUE,
	"EnumSignalEnumeratedFlagValue", // OSF32_ENUMSIGNALENUMERATEDFLAGVALUE,
	"EnumDropEnumeratedFlagValue", // OSF32_ENUMDROPENUMERATEDFLAGVALUE,
	"EnumToggleEnumeratedFlagValue", // OSF32_ENUMTOGGLEENUMERATEDFLAGVALUE,
	"EnumModifyEnumeratedFlagValue", // OSF32_ENUMMODIFYENUMERATEDFLAGVALUE,
	"EnumSignalFirstEnumeratedFlagValue", // OSF32_ENUMSIGNALFIRSTENUMERATEDFLAGVALUE,
	"EnumSignalLastEnumeratedFlagValue", // OSF32_ENUMSIGNALLASTENUMERATEDFLAGVALUE,
	"EnumGetEnumeratedFlagValue", // OSF32_ENUMGETENUMERATEDFLAGVALUE,
	"EnumFindFirstEnumeratedFlag", // OSF32_ENUMFINDFIRSTENUMERATEDFLAG,
	"EnumAllSignalEnumeratedFlags", // OSF32_ENUMALLSIGNALENUMERATEDFLAGS,
	"EnumAllDropEnumeratedFlags", // OSF32_ENUMALLDROPENUMERATEDFLAGS,
	"EnumAllQueryEnumeratedFlags", // OSF32_ENUMALLQUERYENUMERATEDFLAGS,
	"EnumAnyGetEnumeratedFlagValue", // OSF32_ENUMANYGETENUMERATEDFLAGVALUE,
	"StoreFlagsEnumeratedValue", // OSF32_STOREFLAGSENUMERATEDVALUE,
	"RetrieveFlagsEnumeratedValue", // OSF32_RETRIEVEFLAGSENUMERATEDVALUE,
};
static const CEnumUnsortedElementArray<EOUSIMPLEFLAGSFEATURE32, OSF32__MAX, const char *> g_aszSimpleFlags32FeatureTestNames;


bool TestSimpleFlags32(unsigned int &nOutSuccessCount, unsigned int &nOutTestCount)
{
	return TestSubsystem(nOutSuccessCount, nOutTestCount, OSF32__MAX, g_aszSimpleFlags32FeatureTestNames.GetStoragePointer(), g_afnSimpleFlags32FeatureTestProcedures.GetStoragePointer());
}


//////////////////////////////////////////////////////////////////////////

typedef CSimpleFlagsTemplate<uint16ou> CSimpleFlags16;

const uint16ou g_uiTestValue16 = (uint16ou)0xA5A5;
const uint16ou g_uiTestMask16 = (uint16ou)0xC6C6;
const uint16ou g_uiTestBit16 = (uint16ou)OU_INT16_MIN;
const uint16ou g_uiTestAnotherBit16 = (uint16ou)((uint16ou)OU_INT16_MIN >> 1);


bool TestSimpleFlags16_Constructors()
{
	bool bResult = false;
	
	do
	{
		if (sizeof(CSimpleFlags16::value_type) != sizeof(uint16ou) || sizeof(CSimpleFlags16) != sizeof(uint16ou))
		{
			break;
		}

		CSimpleFlags16 sfEmptyFlags;

		if (sfEmptyFlags.QueryFlagsAllValues())
		{
			break;
		}

		CSimpleFlags16 sfFullFlags(OU_UINT16_MAX);
		
		if (sfFullFlags.QueryFlagsAllValues() != OU_UINT16_MAX)
		{
			break;
		}

		CSimpleFlags16 sfCopyOfFullFlags(sfFullFlags);

		if (sfCopyOfFullFlags.QueryFlagsAllValues() != OU_UINT16_MAX)
		{
			break;
		}
	
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags16_AssignFlagsAllValues()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags16 sfTestFlags;

		sfTestFlags.AssignFlagsAllValues(OU_UINT16_MAX);

		if (sfTestFlags.QueryFlagsAllValues() != OU_UINT16_MAX)
		{
			break;
		}

		sfTestFlags.AssignFlagsAllValues(0);

		if (sfTestFlags.QueryFlagsAllValues() != 0)
		{
			break;
		}
	
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags16_QueryFlagsAllValues()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags16 sfTestFlags(g_uiTestValue16);
		
		if (sfTestFlags.QueryFlagsAllValues() != g_uiTestValue16)
		{
			break;
		}

		// Double check to be sure ;-)
		if (sfTestFlags.QueryFlagsAllValues() != g_uiTestValue16)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags16_SetFlagsMaskValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags16 sfTestFlags(g_uiTestValue16);

		sfTestFlags.SetFlagsMaskValue(g_uiTestMask16, true);

		if (sfTestFlags.QueryFlagsAllValues() != (uint16ou)(g_uiTestValue16 | g_uiTestMask16))
		{
			break;
		}

		sfTestFlags.SetFlagsMaskValue(g_uiTestValue16, false);

		if (sfTestFlags.QueryFlagsAllValues() != (uint16ou)(~g_uiTestValue16 & g_uiTestMask16))
		{
			break;
		}
		

		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags16_SignalFlagsMaskValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags16 sfTestFlags(g_uiTestValue16);
		
		sfTestFlags.SignalFlagsMaskValue(g_uiTestMask16);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint16ou)(g_uiTestValue16 | g_uiTestMask16))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags16_DropFlagsMaskValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags16 sfTestFlags(g_uiTestValue16);
		
		sfTestFlags.DropFlagsMaskValue(g_uiTestMask16);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint16ou)(g_uiTestValue16 & ~g_uiTestMask16))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags16_ToggleSingleFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags16 sfTestFlags(g_uiTestValue16);
		
		bool bPreviousValue = sfTestFlags.ToggleSingleFlagValue(g_uiTestBit16);
		
		if (bPreviousValue != ((g_uiTestValue16 & g_uiTestBit16) != 0) || sfTestFlags.QueryFlagsAllValues() != (uint16ou)(g_uiTestValue16 ^ g_uiTestBit16))
		{
			break;
		}

		bool bAnotherPreviousValue = sfTestFlags.ToggleSingleFlagValue(g_uiTestBit16);
		
		if (bAnotherPreviousValue == bPreviousValue || sfTestFlags.QueryFlagsAllValues() != g_uiTestValue16)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags16_ModifySingleFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags16 sfTestFlags(g_uiTestValue16);
		
		bool bFirstModification = sfTestFlags.ModifySingleFlagValue(g_uiTestBit16, true);
		
		if (bFirstModification != ((g_uiTestValue16 & g_uiTestBit16) != g_uiTestBit16) || sfTestFlags.QueryFlagsAllValues() != (uint16ou)(g_uiTestValue16 | g_uiTestBit16))
		{
			break;
		}
		
		bool bAnotherModification = sfTestFlags.ModifySingleFlagValue(g_uiTestBit16, bFirstModification);
		
		if (bAnotherModification == bFirstModification || sfTestFlags.QueryFlagsAllValues() != (bFirstModification ? (uint16ou)(g_uiTestValue16 | g_uiTestBit16) : (uint16ou)(g_uiTestValue16 & ~g_uiTestBit16)))
		{
			break;
		}

		bool bYetAnotherModification = sfTestFlags.ModifySingleFlagValue(g_uiTestBit16, bAnotherModification);

		if (bYetAnotherModification != bAnotherModification || sfTestFlags.QueryFlagsAllValues() != (bAnotherModification ? (uint16ou)(g_uiTestValue16 | g_uiTestBit16) : (uint16ou)(g_uiTestValue16 & ~g_uiTestBit16)))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags16_AssignFlagsByMask()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags16 sfTestFlags(g_uiTestValue16);

		uint16ou uiPreviousFlags = sfTestFlags.AssignFlagsByMask(g_uiTestMask16, g_uiTestMask16);

		const uint16ou uiNewFlags = (g_uiTestValue16 & ~g_uiTestMask16) | g_uiTestMask16;

		if (uiPreviousFlags != g_uiTestValue16 || sfTestFlags.QueryFlagsAllValues() != uiNewFlags)
		{
			break;
		}

		uint16ou uiAnotherPreviousFlags = sfTestFlags.AssignFlagsByMask(g_uiTestValue16, 0);

		const uint16ou uiAnotherNewFlags = uiNewFlags & ~g_uiTestValue16;

		if (uiAnotherPreviousFlags != uiNewFlags || sfTestFlags.QueryFlagsAllValues() != uiAnotherNewFlags)
		{
			break;
		}
	
		uint16ou uiYetAnotherPreviousFlags = sfTestFlags.AssignFlagsByMask(g_uiTestMask16, g_uiTestMask16 & g_uiTestValue16);
		OU_ASSERT((g_uiTestMask16 & g_uiTestValue16) != 0); // Test degeneration
		
		const uint16ou uiYetAnotherNewFlags = (uiAnotherNewFlags & ~g_uiTestMask16) | (g_uiTestMask16 & g_uiTestValue16);
		OU_ASSERT(uiYetAnotherNewFlags != OU_UINT16_MAX); // Test degeneration
		
		if (uiYetAnotherPreviousFlags != uiAnotherNewFlags || sfTestFlags.QueryFlagsAllValues() != uiYetAnotherNewFlags)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags16_AlterFlagsByMask()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags16 sfTestFlags(g_uiTestValue16);
		
		bool bWasModification = sfTestFlags.AlterFlagsByMask(g_uiTestMask16, g_uiTestMask16);
		
		const uint16ou uiNewFlags = (g_uiTestValue16 & ~g_uiTestMask16) | g_uiTestMask16;
		
		if (bWasModification != ((g_uiTestValue16 & g_uiTestMask16) != g_uiTestMask16) || sfTestFlags.QueryFlagsAllValues() != uiNewFlags)
		{
			break;
		}
		
		bool bWasAnotherModification = sfTestFlags.AlterFlagsByMask(g_uiTestValue16, 0);
		
		const uint16ou uiAnotherNewFlags = uiNewFlags & ~g_uiTestValue16;
		
		if (bWasAnotherModification != ((uiNewFlags & g_uiTestValue16) != 0) || sfTestFlags.QueryFlagsAllValues() != uiAnotherNewFlags)
		{
			break;
		}
		
		bool bWasAnotherModificationRepeated = sfTestFlags.AlterFlagsByMask(g_uiTestValue16, 0);
		
		if (bWasAnotherModificationRepeated || sfTestFlags.QueryFlagsAllValues() != uiAnotherNewFlags)
		{
			break;
		}
		
		bool bWasYetAnotherModification = sfTestFlags.AlterFlagsByMask(g_uiTestMask16, g_uiTestMask16 & g_uiTestValue16);
		OU_ASSERT((g_uiTestMask16 & g_uiTestValue16) != 0); // Test degeneration
		
		const uint16ou uiYetAnotherNewFlags = (uiAnotherNewFlags & ~g_uiTestMask16) | (g_uiTestMask16 & g_uiTestValue16);
		OU_ASSERT(uiYetAnotherNewFlags != OU_UINT16_MAX); // Test degeneration
		
		if (bWasYetAnotherModification != ((uiAnotherNewFlags & g_uiTestMask16) != (g_uiTestMask16 & g_uiTestValue16)) || sfTestFlags.QueryFlagsAllValues() != uiYetAnotherNewFlags)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags16_GetFlagsMaskValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags16 sfTestFlags(g_uiTestValue16);
		
		if (sfTestFlags.GetFlagsMaskValue(g_uiTestMask16) != ((g_uiTestValue16 & g_uiTestMask16) != 0))
		{
			break;
		}
		
		if (sfTestFlags.GetFlagsMaskValue((uint16ou)(~g_uiTestValue16)))
		{
			break;
		}
		
		if (!sfTestFlags.GetFlagsMaskValue(OU_UINT16_MAX))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags16_QueryFlagsByMask()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags16 sfTestFlags(g_uiTestValue16);
		
		if (sfTestFlags.QueryFlagsByMask(g_uiTestMask16) != (uint16ou)(g_uiTestValue16 & g_uiTestMask16))
		{
			break;
		}
		
		if (sfTestFlags.QueryFlagsByMask(0))
		{
			break;
		}
		
		if (sfTestFlags.QueryFlagsByMask(OU_UINT16_MAX) != g_uiTestValue16)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags16_OnlySignalSingleFlagOutOfMask()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags16 sfTestFlags(g_uiTestValue16);
		OU_ASSERT(g_uiTestValue16 != 0); // Test degeneration
		
		if (sfTestFlags.OnlySignalSingleFlagOutOfMask(OU_UINT16_MAX, g_uiTestBit16))
		{
			break;
		}

		if (sfTestFlags.QueryFlagsAllValues() != g_uiTestValue16)
		{
			break;
		}
		
		sfTestFlags.AssignFlagsAllValues(0);

		if (!sfTestFlags.OnlySignalSingleFlagOutOfMask(g_uiTestBit16, g_uiTestBit16))
		{
			break;
		}
		
		if (sfTestFlags.QueryFlagsAllValues() != g_uiTestBit16)
		{
			break;
		}
		
		if (sfTestFlags.OnlySignalSingleFlagOutOfMask(g_uiTestBit16, g_uiTestBit16))
		{
			break;
		}
		
		if (sfTestFlags.QueryFlagsAllValues() != g_uiTestBit16)
		{
			break;
		}
		
		if (sfTestFlags.OnlySignalSingleFlagOutOfMask(OU_UINT16_MAX, g_uiTestAnotherBit16))
		{
			break;
		}
		
		if (sfTestFlags.QueryFlagsAllValues() != g_uiTestBit16)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags16_EnumSetEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags16 sfTestFlags;
		
		sfTestFlags.EnumSetEnumeratedFlagValue(1, 0, OU_UINT16_BITS, true);

		if (sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
	
		sfTestFlags.EnumSetEnumeratedFlagValue(1, OU_UINT16_BITS - 1, OU_UINT16_BITS, true);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint16ou)(OU_INT16_MIN + 1))
		{
			break;
		}
		
		sfTestFlags.EnumSetEnumeratedFlagValue(1, 0, OU_UINT16_BITS, false);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint16ou)OU_INT16_MIN)
		{
			break;
		}
		
		sfTestFlags.EnumSetEnumeratedFlagValue(1, OU_UINT16_BITS - 1, OU_UINT16_BITS, false);
		
		if (sfTestFlags.QueryFlagsAllValues() != 0)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags16_EnumSignalEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags16 sfTestFlags;
		
		sfTestFlags.EnumSignalEnumeratedFlagValue(1, 0, OU_UINT16_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		sfTestFlags.EnumSignalEnumeratedFlagValue(1, OU_UINT16_BITS - 1, OU_UINT16_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint16ou)(OU_INT16_MIN + 1))
		{
			break;
		}
		
		sfTestFlags.EnumSignalEnumeratedFlagValue(1, 0, OU_UINT16_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint16ou)(OU_INT16_MIN + 1))
		{
			break;
		}
		
		sfTestFlags.EnumSignalEnumeratedFlagValue(1, OU_UINT16_BITS - 1, OU_UINT16_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint16ou)(OU_INT16_MIN + 1))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags16_EnumDropEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags16 sfTestFlags(OU_UINT16_MAX);
		
		sfTestFlags.EnumDropEnumeratedFlagValue(1, 0, OU_UINT16_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != (OU_UINT16_MAX ^ 1))
		{
			break;
		}
		
		sfTestFlags.EnumDropEnumeratedFlagValue(1, OU_UINT16_BITS - 1, OU_UINT16_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint16ou)~(OU_INT16_MIN + 1))
		{
			break;
		}
		
		sfTestFlags.EnumDropEnumeratedFlagValue(1, 0, OU_UINT16_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint16ou)~(OU_INT16_MIN + 1))
		{
			break;
		}
		
		sfTestFlags.EnumDropEnumeratedFlagValue(1, OU_UINT16_BITS - 1, OU_UINT16_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint16ou)~(OU_INT16_MIN + 1))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags16_EnumToggleEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags16 sfTestFlags;
		
		bool bToggleFirstResult = sfTestFlags.EnumToggleEnumeratedFlagValue(1, 0, OU_UINT16_BITS);
		
		if (bToggleFirstResult || sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		bool bToggleSecondResult = sfTestFlags.EnumToggleEnumeratedFlagValue(1, OU_UINT16_BITS - 1, OU_UINT16_BITS);
		
		if (bToggleSecondResult || sfTestFlags.QueryFlagsAllValues() != (uint16ou)(OU_INT16_MIN + 1))
		{
			break;
		}
		
		bool bToggleThirdResult = sfTestFlags.EnumToggleEnumeratedFlagValue(1, 0, OU_UINT16_BITS);
		
		if (!bToggleThirdResult || sfTestFlags.QueryFlagsAllValues() != (uint16ou)OU_INT16_MIN)
		{
			break;
		}
		
		bool bToggleFourthResult = sfTestFlags.EnumToggleEnumeratedFlagValue(1, OU_UINT16_BITS - 1, OU_UINT16_BITS);
		
		if (!bToggleFourthResult || sfTestFlags.QueryFlagsAllValues() != 0)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags16_EnumModifyEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags16 sfTestFlags;
		
		bool bModifyFirstResult = sfTestFlags.EnumModifyEnumeratedFlagValue(1, 0, OU_UINT16_BITS, true);
		
		if (!bModifyFirstResult || sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		bool bModifySecondResult = sfTestFlags.EnumModifyEnumeratedFlagValue(1, OU_UINT16_BITS - 1, OU_UINT16_BITS, true);
		
		if (!bModifySecondResult || sfTestFlags.QueryFlagsAllValues() != (uint16ou)(OU_INT16_MIN + 1))
		{
			break;
		}
		
		bool bModifyThirdResult = sfTestFlags.EnumModifyEnumeratedFlagValue(1, 0, OU_UINT16_BITS, true);
		
		if (bModifyThirdResult || sfTestFlags.QueryFlagsAllValues() != (uint16ou)(OU_INT16_MIN + 1))
		{
			break;
		}
		
		bool bModifyFourthResult = sfTestFlags.EnumModifyEnumeratedFlagValue(1, OU_UINT16_BITS - 1, OU_UINT16_BITS, true);
		
		if (bModifyFourthResult || sfTestFlags.QueryFlagsAllValues() != (uint16ou)(OU_INT16_MIN + 1))
		{
			break;
		}
		
		bool bModifyFifthResult = sfTestFlags.EnumModifyEnumeratedFlagValue(1, 0, OU_UINT16_BITS, false);
		
		if (!bModifyFifthResult || sfTestFlags.QueryFlagsAllValues() != (uint16ou)OU_INT16_MIN)
		{
			break;
		}
		
		bool bModifySixthResult = sfTestFlags.EnumModifyEnumeratedFlagValue(1, OU_UINT16_BITS - 1, OU_UINT16_BITS, false);
		
		if (!bModifySixthResult || sfTestFlags.QueryFlagsAllValues() != 0)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags16_EnumSignalFirstEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags16 sfTestFlags;

		bool bFirstResult = sfTestFlags.EnumSignalFirstEnumeratedFlagValue(1, 0, OU_UINT16_BITS);

		if (!bFirstResult || sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		bool bSecondResult = sfTestFlags.EnumSignalFirstEnumeratedFlagValue(1, 0, OU_UINT16_BITS);
		
		if (bSecondResult || sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		bool bThirdResult = sfTestFlags.EnumSignalFirstEnumeratedFlagValue(2, 0, OU_UINT16_BITS - 1);
		
		if (!bThirdResult || sfTestFlags.QueryFlagsAllValues() != 3)
		{
			break;
		}
		
		bool bFourthResult = sfTestFlags.EnumSignalFirstEnumeratedFlagValue(1, OU_UINT16_BITS - 1, OU_UINT16_BITS);
		
		if (bFourthResult || sfTestFlags.QueryFlagsAllValues() != (uint16ou)(OU_INT16_MIN + 3))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags16_EnumSignalLastEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags16 sfTestFlags;
		
		bool bFirstResult = sfTestFlags.EnumSignalLastEnumeratedFlagValue(1, 0, 1);
		
		if (!bFirstResult || sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		bool bSecondResult = sfTestFlags.EnumSignalLastEnumeratedFlagValue(1, 0, 1);
		
		if (bSecondResult || sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		bool bThirdResult = sfTestFlags.EnumSignalLastEnumeratedFlagValue(1, 1, 2);
		
		if (!bThirdResult || sfTestFlags.QueryFlagsAllValues() != 3)
		{
			break;
		}
		
		bool bFourthResult = sfTestFlags.EnumSignalLastEnumeratedFlagValue(1, OU_UINT16_BITS - 1, OU_UINT16_BITS);
		
		if (bFourthResult || sfTestFlags.QueryFlagsAllValues() != (uint16ou)(OU_INT16_MIN + 3))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags16_EnumGetEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags16 sfTestFlags((uint16ou)(OU_INT16_MIN + 1));
		
		if (!sfTestFlags.EnumGetEnumeratedFlagValue(1, 0, OU_UINT16_BITS))
		{
			break;
		}
	
		if (sfTestFlags.EnumGetEnumeratedFlagValue(2, 0, OU_UINT16_BITS - 1))
		{
			break;
		}
		
		if (sfTestFlags.EnumGetEnumeratedFlagValue(1, 1, OU_UINT16_BITS - 1))
		{
			break;
		}
		
		if (!sfTestFlags.EnumGetEnumeratedFlagValue(1, OU_UINT16_BITS - 1, OU_UINT16_BITS))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags16_EnumFindFirstEnumeratedFlag()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags16 sfTestFlags((uint16ou)(OU_INT16_MIN + 1));

		unsigned int uiFirstResult = sfTestFlags.EnumFindFirstEnumeratedFlag(1, OU_UINT16_BITS);
		if (uiFirstResult != 0)
		{
			break;
		}

		unsigned int uiSecondResult = sfTestFlags.EnumFindFirstEnumeratedFlag(2, OU_UINT16_BITS - 1);
		if (uiSecondResult != OU_UINT16_BITS - 2)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags16_EnumAllSignalEnumeratedFlags()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags16 sfTestFlags;
		
		sfTestFlags.EnumAllSignalEnumeratedFlags(1, 1);

		if (sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}

		sfTestFlags.EnumAllSignalEnumeratedFlags(4, OU_UINT16_BITS - 2);

		if (sfTestFlags.QueryFlagsAllValues() != (uint16ou)(OU_UINT16_MAX ^ 2))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags16_EnumAllDropEnumeratedFlags()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags16 sfTestFlags(OU_UINT16_MAX);
		
		sfTestFlags.EnumAllDropEnumeratedFlags(1, 1);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint16ou)(OU_UINT16_MAX ^ 1))
		{
			break;
		}
		
		sfTestFlags.EnumAllDropEnumeratedFlags(4, OU_UINT16_BITS - 2);
		
		if (sfTestFlags.QueryFlagsAllValues() != 2)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags16_EnumAllQueryEnumeratedFlags()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags16 sfTestFlags((uint16ou)(OU_INT16_MIN + 1));
		
		uint16ou uiFirstResult = sfTestFlags.EnumAllQueryEnumeratedFlags(1, OU_UINT16_BITS);
		if (uiFirstResult != (uint16ou)(OU_INT16_MIN + 1))
		{
			break;
		}
		
		uint16ou uiSecondResult = sfTestFlags.EnumAllQueryEnumeratedFlags(2, OU_UINT16_BITS - 1);
		if (uiSecondResult != (uint16ou)(OU_INT16_MIN))
		{
			break;
		}
		
		uint16ou uiThirdResult = sfTestFlags.EnumAllQueryEnumeratedFlags(2, OU_UINT16_BITS - 2);
		if (uiThirdResult != 0)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags16_EnumAnyGetEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags16 sfTestFlags((uint16ou)(OU_INT16_MIN + 1));
		
		bool bFirstResult = sfTestFlags.EnumAnyGetEnumeratedFlagValue(1, OU_UINT16_BITS);
		if (!bFirstResult)
		{
			break;
		}
		
		bool bSecondResult = sfTestFlags.EnumAnyGetEnumeratedFlagValue(2, OU_UINT16_BITS - 1);
		if (!bSecondResult)
		{
			break;
		}
		
		bool bThirdResult = sfTestFlags.EnumAnyGetEnumeratedFlagValue(2, OU_UINT16_BITS - 2);
		if (bThirdResult)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags16_StoreFlagsEnumeratedValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags16 sfTestFlags;

		sfTestFlags.StoreFlagsEnumeratedValue(0x03, 1, 2);

		if (sfTestFlags.QueryFlagsAllValues() != (uint16ou)(2 << 1))
		{
			break;
		}
	
		sfTestFlags.StoreFlagsEnumeratedValue(0x03, OU_UINT16_BITS - 2, 3);
		
		if (sfTestFlags.QueryFlagsAllValues() != ((uint16ou)(2 << 1) | (uint16ou)(OU_INT16_MIN | (OU_INT16_MIN >> 1))))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags16_RetrieveFlagsEnumeratedValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags16 sfTestFlags((uint16ou)(OU_INT16_MIN + 1));
		
		unsigned int uiFirstResult = sfTestFlags.RetrieveFlagsEnumeratedValue(0x3, 1);
		if (uiFirstResult != 0)
		{
			break;
		}
		
		unsigned int uiSecondResult = sfTestFlags.RetrieveFlagsEnumeratedValue(0x3, OU_UINT16_BITS - 2);
		if (uiSecondResult != 2)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}


enum EOUSIMPLEFLAGSFEATURE16
{
	OSF16__MIN,
		
	OSF16_CONSTRUCTORS = OSF16__MIN,
	OSF16_ASSIGNFLAGSALLVALUES,
	OSF16_QUERYFLAGSALLVALUES,
	OSF16_SETFLAGSMASKVALUE,
	OSF16_SIGNALFLAGSMASKVALUE,
	OSF16_DROPFLAGSMASKVALUE,
	OSF16_TOGGLESINGLEFLAGVALUE,
	OSF16_MODIFYSINGLEFLAGVALUE,
	OSF16_ASSIGNFLAGSBYMASK,
	OSF16_ALTERFLAGSBYMASK,
	OSF16_GETFLAGSMASKVALUE,
	OSF16_QUERYFLAGSBYMASK,
	OSF16_ONLYSIGNALSINGLEFLAGOUTOFMASK,
	OSF16_ENUMSETENUMERATEDFLAGVALUE,
	OSF16_ENUMSIGNALENUMERATEDFLAGVALUE,
	OSF16_ENUMDROPENUMERATEDFLAGVALUE,
	OSF16_ENUMTOGGLEENUMERATEDFLAGVALUE,
	OSF16_ENUMMODIFYENUMERATEDFLAGVALUE,
	OSF16_ENUMSIGNALFIRSTENUMERATEDFLAGVALUE,
	OSF16_ENUMSIGNALLASTENUMERATEDFLAGVALUE,
	OSF16_ENUMGETENUMERATEDFLAGVALUE,
	OSF16_ENUMFINDFIRSTENUMERATEDFLAG,
	OSF16_ENUMALLSIGNALENUMERATEDFLAGS,
	OSF16_ENUMALLDROPENUMERATEDFLAGS,
	OSF16_ENUMALLQUERYENUMERATEDFLAGS,
	OSF16_ENUMANYGETENUMERATEDFLAGVALUE,
	OSF16_STOREFLAGSENUMERATEDVALUE,
	OSF16_RETRIEVEFLAGSENUMERATEDVALUE,

	OSF16__MAX,
};

template<>
CFeatureTestProcedure const CEnumUnsortedElementArray<EOUSIMPLEFLAGSFEATURE16, OSF16__MAX, CFeatureTestProcedure>::m_aetElementArray[] =
{
	&TestSimpleFlags16_Constructors, // OSF16_CONSTRUCTORS
	&TestSimpleFlags16_AssignFlagsAllValues, // OSF16_ASSIGNFLAGSALLVALUES,
	&TestSimpleFlags16_QueryFlagsAllValues, // OSF16_QUERYFLAGSALLVALUES,
	&TestSimpleFlags16_SetFlagsMaskValue, // OSF16_SETFLAGSMASKVALUE,
	&TestSimpleFlags16_SignalFlagsMaskValue, // OSF16_SIGNALFLAGSMASKVALUE,
	&TestSimpleFlags16_DropFlagsMaskValue, // OSF16_DROPFLAGSMASKVALUE,
	&TestSimpleFlags16_ToggleSingleFlagValue, // OSF16_TOGGLESINGLEFLAGVALUE,
	&TestSimpleFlags16_ModifySingleFlagValue, // OSF16_MODIFYSINGLEFLAGVALUE,
	&TestSimpleFlags16_AssignFlagsByMask, // OSF16_ASSIGNFLAGSBYMASK,
	&TestSimpleFlags16_AlterFlagsByMask, // OSF16_ALTERFLAGSBYMASK,
	&TestSimpleFlags16_GetFlagsMaskValue, // OSF16_GETFLAGSMASKVALUE,
	&TestSimpleFlags16_QueryFlagsByMask, // OSF16_QUERYFLAGSBYMASK,
	&TestSimpleFlags16_OnlySignalSingleFlagOutOfMask, // OSF16_ONLYSIGNALSINGLEFLAGOUTOFMASK,
	&TestSimpleFlags16_EnumSetEnumeratedFlagValue, // OSF16_ENUMSETENUMERATEDFLAGVALUE,
	&TestSimpleFlags16_EnumSignalEnumeratedFlagValue, // OSF16_ENUMSIGNALENUMERATEDFLAGVALUE,
	&TestSimpleFlags16_EnumDropEnumeratedFlagValue, // OSF16_ENUMDROPENUMERATEDFLAGVALUE,
	&TestSimpleFlags16_EnumToggleEnumeratedFlagValue, // OSF16_ENUMTOGGLEENUMERATEDFLAGVALUE,
	&TestSimpleFlags16_EnumModifyEnumeratedFlagValue, // OSF16_ENUMMODIFYENUMERATEDFLAGVALUE,
	&TestSimpleFlags16_EnumSignalFirstEnumeratedFlagValue, // OSF16_ENUMSIGNALFIRSTENUMERATEDFLAGVALUE,
	&TestSimpleFlags16_EnumSignalLastEnumeratedFlagValue, // OSF16_ENUMSIGNALLASTENUMERATEDFLAGVALUE,
	&TestSimpleFlags16_EnumGetEnumeratedFlagValue, // OSF16_ENUMGETENUMERATEDFLAGVALUE,
	&TestSimpleFlags16_EnumFindFirstEnumeratedFlag, // OSF16_ENUMFINDFIRSTENUMERATEDFLAG,
	&TestSimpleFlags16_EnumAllSignalEnumeratedFlags, // OSF16_ENUMALLSIGNALENUMERATEDFLAGS,
	&TestSimpleFlags16_EnumAllDropEnumeratedFlags, // OSF16_ENUMALLDROPENUMERATEDFLAGS,
	&TestSimpleFlags16_EnumAllQueryEnumeratedFlags, // OSF16_ENUMALLQUERYENUMERATEDFLAGS,
	&TestSimpleFlags16_EnumAnyGetEnumeratedFlagValue, // OSF16_ENUMANYGETENUMERATEDFLAGVALUE,
	&TestSimpleFlags16_StoreFlagsEnumeratedValue, // OSF16_STOREFLAGSENUMERATEDVALUE,
	&TestSimpleFlags16_RetrieveFlagsEnumeratedValue, // OSF16_RETRIEVEFLAGSENUMERATEDVALUE,
};
static const CEnumUnsortedElementArray<EOUSIMPLEFLAGSFEATURE16, OSF16__MAX, CFeatureTestProcedure> g_afnSimpleFlags16FeatureTestProcedures;

template<>
const char *const CEnumUnsortedElementArray<EOUSIMPLEFLAGSFEATURE16, OSF16__MAX, const char *>::m_aetElementArray[] =
{
	"Constructors", // OSF16_CONSTRUCTORS
	"AssignFlagsAllValues", // OSF16_ASSIGNFLAGSALLVALUES,
	"QueryFlagsAllValues", // OSF16_QUERYFLAGSALLVALUES,
	"SetFlagsMaskValue", // OSF16_SETFLAGSMASKVALUE,
	"SignalFlagsMaskValue", // OSF16_SIGNALFLAGSMASKVALUE,
	"DropFlagsMaskValue", // OSF16_DROPFLAGSMASKVALUE,
	"ToggleSingleFlagValue", // OSF16_TOGGLESINGLEFLAGVALUE,
	"ModifySingleFlagValue", // OSF16_MODIFYSINGLEFLAGVALUE,
	"AssignFlagsByMask", // OSF16_ASSIGNFLAGSBYMASK,
	"AlterFlagsByMask", // OSF16_ALTERFLAGSBYMASK,
	"GetFlagsMaskValue", // OSF16_GETFLAGSMASKVALUE,
	"QueryFlagsByMask", // OSF16_QUERYFLAGSBYMASK,
	"OnlySignalSingleFlagOutOfMask", // OSF16_ONLYSIGNALSINGLEFLAGOUTOFMASK,
	"EnumSetEnumeratedFlagValue", // OSF16_ENUMSETENUMERATEDFLAGVALUE,
	"EnumSignalEnumeratedFlagValue", // OSF16_ENUMSIGNALENUMERATEDFLAGVALUE,
	"EnumDropEnumeratedFlagValue", // OSF16_ENUMDROPENUMERATEDFLAGVALUE,
	"EnumToggleEnumeratedFlagValue", // OSF16_ENUMTOGGLEENUMERATEDFLAGVALUE,
	"EnumModifyEnumeratedFlagValue", // OSF16_ENUMMODIFYENUMERATEDFLAGVALUE,
	"EnumSignalFirstEnumeratedFlagValue", // OSF16_ENUMSIGNALFIRSTENUMERATEDFLAGVALUE,
	"EnumSignalLastEnumeratedFlagValue", // OSF16_ENUMSIGNALLASTENUMERATEDFLAGVALUE,
	"EnumGetEnumeratedFlagValue", // OSF16_ENUMGETENUMERATEDFLAGVALUE,
	"EnumFindFirstEnumeratedFlag", // OSF16_ENUMFINDFIRSTENUMERATEDFLAG,
	"EnumAllSignalEnumeratedFlags", // OSF16_ENUMALLSIGNALENUMERATEDFLAGS,
	"EnumAllDropEnumeratedFlags", // OSF16_ENUMALLDROPENUMERATEDFLAGS,
	"EnumAllQueryEnumeratedFlags", // OSF16_ENUMALLQUERYENUMERATEDFLAGS,
	"EnumAnyGetEnumeratedFlagValue", // OSF16_ENUMANYGETENUMERATEDFLAGVALUE,
	"StoreFlagsEnumeratedValue", // OSF16_STOREFLAGSENUMERATEDVALUE,
	"RetrieveFlagsEnumeratedValue", // OSF16_RETRIEVEFLAGSENUMERATEDVALUE,
};
static const CEnumUnsortedElementArray<EOUSIMPLEFLAGSFEATURE16, OSF16__MAX, const char *> g_aszSimpleFlags16FeatureTestNames;


bool TestSimpleFlags16(unsigned int &nOutSuccessCount, unsigned int &nOutTestCount)
{
	return TestSubsystem(nOutSuccessCount, nOutTestCount, OSF16__MAX, g_aszSimpleFlags16FeatureTestNames.GetStoragePointer(), g_afnSimpleFlags16FeatureTestProcedures.GetStoragePointer());
}


//////////////////////////////////////////////////////////////////////////

typedef CSimpleFlagsTemplate<uint8ou> CSimpleFlags8;

const uint8ou g_uiTestValue8 = (uint8ou)0xA5;
const uint8ou g_uiTestMask8 = (uint8ou)0xC6;
const uint8ou g_uiTestBit8 = (uint8ou)OU_INT8_MIN;
const uint8ou g_uiTestAnotherBit8 = (uint8ou)((uint8ou)OU_INT8_MIN >> 1);


bool TestSimpleFlags8_Constructors()
{
	bool bResult = false;
	
	do
	{
		if (sizeof(CSimpleFlags8::value_type) != sizeof(uint8ou) || sizeof(CSimpleFlags8) != sizeof(uint8ou))
		{
			break;
		}

		CSimpleFlags8 sfEmptyFlags;

		if (sfEmptyFlags.QueryFlagsAllValues())
		{
			break;
		}

		CSimpleFlags8 sfFullFlags(OU_UINT8_MAX);
		
		if (sfFullFlags.QueryFlagsAllValues() != OU_UINT8_MAX)
		{
			break;
		}

		CSimpleFlags8 sfCopyOfFullFlags(sfFullFlags);

		if (sfCopyOfFullFlags.QueryFlagsAllValues() != OU_UINT8_MAX)
		{
			break;
		}
	
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags8_AssignFlagsAllValues()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags8 sfTestFlags;

		sfTestFlags.AssignFlagsAllValues(OU_UINT8_MAX);

		if (sfTestFlags.QueryFlagsAllValues() != OU_UINT8_MAX)
		{
			break;
		}

		sfTestFlags.AssignFlagsAllValues(0);

		if (sfTestFlags.QueryFlagsAllValues() != 0)
		{
			break;
		}
	
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags8_QueryFlagsAllValues()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags8 sfTestFlags(g_uiTestValue8);
		
		if (sfTestFlags.QueryFlagsAllValues() != g_uiTestValue8)
		{
			break;
		}

		// Double check to be sure ;-)
		if (sfTestFlags.QueryFlagsAllValues() != g_uiTestValue8)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags8_SetFlagsMaskValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags8 sfTestFlags(g_uiTestValue8);

		sfTestFlags.SetFlagsMaskValue(g_uiTestMask8, true);

		if (sfTestFlags.QueryFlagsAllValues() != (uint8ou)(g_uiTestValue8 | g_uiTestMask8))
		{
			break;
		}

		sfTestFlags.SetFlagsMaskValue(g_uiTestValue8, false);

		if (sfTestFlags.QueryFlagsAllValues() != (uint8ou)(~g_uiTestValue8 & g_uiTestMask8))
		{
			break;
		}
		

		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags8_SignalFlagsMaskValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags8 sfTestFlags(g_uiTestValue8);
		
		sfTestFlags.SignalFlagsMaskValue(g_uiTestMask8);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint8ou)(g_uiTestValue8 | g_uiTestMask8))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags8_DropFlagsMaskValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags8 sfTestFlags(g_uiTestValue8);
		
		sfTestFlags.DropFlagsMaskValue(g_uiTestMask8);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint8ou)(g_uiTestValue8 & ~g_uiTestMask8))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags8_ToggleSingleFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags8 sfTestFlags(g_uiTestValue8);
		
		bool bPreviousValue = sfTestFlags.ToggleSingleFlagValue(g_uiTestBit8);
		
		if (bPreviousValue != ((g_uiTestValue8 & g_uiTestBit8) != 0) || sfTestFlags.QueryFlagsAllValues() != (uint8ou)(g_uiTestValue8 ^ g_uiTestBit8))
		{
			break;
		}

		bool bAnotherPreviousValue = sfTestFlags.ToggleSingleFlagValue(g_uiTestBit8);
		
		if (bAnotherPreviousValue == bPreviousValue || sfTestFlags.QueryFlagsAllValues() != g_uiTestValue8)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags8_ModifySingleFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags8 sfTestFlags(g_uiTestValue8);
		
		bool bFirstModification = sfTestFlags.ModifySingleFlagValue(g_uiTestBit8, true);
		
		if (bFirstModification != ((g_uiTestValue8 & g_uiTestBit8) != g_uiTestBit8) || sfTestFlags.QueryFlagsAllValues() != (uint8ou)(g_uiTestValue8 | g_uiTestBit8))
		{
			break;
		}
		
		bool bAnotherModification = sfTestFlags.ModifySingleFlagValue(g_uiTestBit8, bFirstModification);
		
		if (bAnotherModification == bFirstModification || sfTestFlags.QueryFlagsAllValues() != (bFirstModification ? (uint8ou)(g_uiTestValue8 | g_uiTestBit8) : (uint8ou)(g_uiTestValue8 & ~g_uiTestBit8)))
		{
			break;
		}

		bool bYetAnotherModification = sfTestFlags.ModifySingleFlagValue(g_uiTestBit8, bAnotherModification);

		if (bYetAnotherModification != bAnotherModification || sfTestFlags.QueryFlagsAllValues() != (bAnotherModification ? (uint8ou)(g_uiTestValue8 | g_uiTestBit8) : (uint8ou)(g_uiTestValue8 & ~g_uiTestBit8)))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags8_AssignFlagsByMask()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags8 sfTestFlags(g_uiTestValue8);

		uint8ou uiPreviousFlags = sfTestFlags.AssignFlagsByMask(g_uiTestMask8, g_uiTestMask8);

		const uint8ou uiNewFlags = (g_uiTestValue8 & ~g_uiTestMask8) | g_uiTestMask8;

		if (uiPreviousFlags != g_uiTestValue8 || sfTestFlags.QueryFlagsAllValues() != uiNewFlags)
		{
			break;
		}

		uint8ou uiAnotherPreviousFlags = sfTestFlags.AssignFlagsByMask(g_uiTestValue8, 0);

		const uint8ou uiAnotherNewFlags = uiNewFlags & ~g_uiTestValue8;

		if (uiAnotherPreviousFlags != uiNewFlags || sfTestFlags.QueryFlagsAllValues() != uiAnotherNewFlags)
		{
			break;
		}
	
		uint8ou uiYetAnotherPreviousFlags = sfTestFlags.AssignFlagsByMask(g_uiTestMask8, g_uiTestMask8 & g_uiTestValue8);
		OU_ASSERT((g_uiTestMask8 & g_uiTestValue8) != 0); // Test degeneration
		
		const uint8ou uiYetAnotherNewFlags = (uiAnotherNewFlags & ~g_uiTestMask8) | (g_uiTestMask8 & g_uiTestValue8);
		OU_ASSERT(uiYetAnotherNewFlags != OU_UINT8_MAX); // Test degeneration
		
		if (uiYetAnotherPreviousFlags != uiAnotherNewFlags || sfTestFlags.QueryFlagsAllValues() != uiYetAnotherNewFlags)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags8_AlterFlagsByMask()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags8 sfTestFlags(g_uiTestValue8);
		
		bool bWasModification = sfTestFlags.AlterFlagsByMask(g_uiTestMask8, g_uiTestMask8);
		
		const uint8ou uiNewFlags = (g_uiTestValue8 & ~g_uiTestMask8) | g_uiTestMask8;
		
		if (bWasModification != ((g_uiTestValue8 & g_uiTestMask8) != g_uiTestMask8) || sfTestFlags.QueryFlagsAllValues() != uiNewFlags)
		{
			break;
		}
		
		bool bWasAnotherModification = sfTestFlags.AlterFlagsByMask(g_uiTestValue8, 0);
		
		const uint8ou uiAnotherNewFlags = uiNewFlags & ~g_uiTestValue8;
		
		if (bWasAnotherModification != ((uiNewFlags & g_uiTestValue8) != 0) || sfTestFlags.QueryFlagsAllValues() != uiAnotherNewFlags)
		{
			break;
		}
		
		bool bWasAnotherModificationRepeated = sfTestFlags.AlterFlagsByMask(g_uiTestValue8, 0);
		
		if (bWasAnotherModificationRepeated || sfTestFlags.QueryFlagsAllValues() != uiAnotherNewFlags)
		{
			break;
		}
		
		bool bWasYetAnotherModification = sfTestFlags.AlterFlagsByMask(g_uiTestMask8, g_uiTestMask8 & g_uiTestValue8);
		OU_ASSERT((g_uiTestMask8 & g_uiTestValue8) != 0); // Test degeneration
		
		const uint8ou uiYetAnotherNewFlags = (uiAnotherNewFlags & ~g_uiTestMask8) | (g_uiTestMask8 & g_uiTestValue8);
		OU_ASSERT(uiYetAnotherNewFlags != OU_UINT8_MAX); // Test degeneration
		
		if (bWasYetAnotherModification != ((uiAnotherNewFlags & g_uiTestMask8) != (g_uiTestMask8 & g_uiTestValue8)) || sfTestFlags.QueryFlagsAllValues() != uiYetAnotherNewFlags)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags8_GetFlagsMaskValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags8 sfTestFlags(g_uiTestValue8);
		
		if (sfTestFlags.GetFlagsMaskValue(g_uiTestMask8) != ((g_uiTestValue8 & g_uiTestMask8) != 0))
		{
			break;
		}
		
		if (sfTestFlags.GetFlagsMaskValue((uint8ou)(~g_uiTestValue8)))
		{
			break;
		}
		
		if (!sfTestFlags.GetFlagsMaskValue(OU_UINT8_MAX))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags8_QueryFlagsByMask()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags8 sfTestFlags(g_uiTestValue8);
		
		if (sfTestFlags.QueryFlagsByMask(g_uiTestMask8) != (uint8ou)(g_uiTestValue8 & g_uiTestMask8))
		{
			break;
		}
		
		if (sfTestFlags.QueryFlagsByMask(0))
		{
			break;
		}
		
		if (sfTestFlags.QueryFlagsByMask(OU_UINT8_MAX) != g_uiTestValue8)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags8_OnlySignalSingleFlagOutOfMask()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags8 sfTestFlags(g_uiTestValue8);
		OU_ASSERT(g_uiTestValue8 != 0); // Test degeneration
		
		if (sfTestFlags.OnlySignalSingleFlagOutOfMask(OU_UINT8_MAX, g_uiTestBit8))
		{
			break;
		}

		if (sfTestFlags.QueryFlagsAllValues() != g_uiTestValue8)
		{
			break;
		}
		
		sfTestFlags.AssignFlagsAllValues(0);

		if (!sfTestFlags.OnlySignalSingleFlagOutOfMask(g_uiTestBit8, g_uiTestBit8))
		{
			break;
		}
		
		if (sfTestFlags.QueryFlagsAllValues() != g_uiTestBit8)
		{
			break;
		}
		
		if (sfTestFlags.OnlySignalSingleFlagOutOfMask(g_uiTestBit8, g_uiTestBit8))
		{
			break;
		}
		
		if (sfTestFlags.QueryFlagsAllValues() != g_uiTestBit8)
		{
			break;
		}
		
		if (sfTestFlags.OnlySignalSingleFlagOutOfMask(OU_UINT8_MAX, g_uiTestAnotherBit8))
		{
			break;
		}
		
		if (sfTestFlags.QueryFlagsAllValues() != g_uiTestBit8)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags8_EnumSetEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags8 sfTestFlags;
		
		sfTestFlags.EnumSetEnumeratedFlagValue(1, 0, OU_UINT8_BITS, true);

		if (sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
	
		sfTestFlags.EnumSetEnumeratedFlagValue(1, OU_UINT8_BITS - 1, OU_UINT8_BITS, true);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint8ou)(OU_INT8_MIN + 1))
		{
			break;
		}
		
		sfTestFlags.EnumSetEnumeratedFlagValue(1, 0, OU_UINT8_BITS, false);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint8ou)OU_INT8_MIN)
		{
			break;
		}
		
		sfTestFlags.EnumSetEnumeratedFlagValue(1, OU_UINT8_BITS - 1, OU_UINT8_BITS, false);
		
		if (sfTestFlags.QueryFlagsAllValues() != 0)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags8_EnumSignalEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags8 sfTestFlags;
		
		sfTestFlags.EnumSignalEnumeratedFlagValue(1, 0, OU_UINT8_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		sfTestFlags.EnumSignalEnumeratedFlagValue(1, OU_UINT8_BITS - 1, OU_UINT8_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint8ou)(OU_INT8_MIN + 1))
		{
			break;
		}
		
		sfTestFlags.EnumSignalEnumeratedFlagValue(1, 0, OU_UINT8_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint8ou)(OU_INT8_MIN + 1))
		{
			break;
		}
		
		sfTestFlags.EnumSignalEnumeratedFlagValue(1, OU_UINT8_BITS - 1, OU_UINT8_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint8ou)(OU_INT8_MIN + 1))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags8_EnumDropEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags8 sfTestFlags(OU_UINT8_MAX);
		
		sfTestFlags.EnumDropEnumeratedFlagValue(1, 0, OU_UINT8_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != (OU_UINT8_MAX ^ 1))
		{
			break;
		}
		
		sfTestFlags.EnumDropEnumeratedFlagValue(1, OU_UINT8_BITS - 1, OU_UINT8_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint8ou)~(OU_INT8_MIN + 1))
		{
			break;
		}
		
		sfTestFlags.EnumDropEnumeratedFlagValue(1, 0, OU_UINT8_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint8ou)~(OU_INT8_MIN + 1))
		{
			break;
		}
		
		sfTestFlags.EnumDropEnumeratedFlagValue(1, OU_UINT8_BITS - 1, OU_UINT8_BITS);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint8ou)~(OU_INT8_MIN + 1))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags8_EnumToggleEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags8 sfTestFlags;
		
		bool bToggleFirstResult = sfTestFlags.EnumToggleEnumeratedFlagValue(1, 0, OU_UINT8_BITS);
		
		if (bToggleFirstResult || sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		bool bToggleSecondResult = sfTestFlags.EnumToggleEnumeratedFlagValue(1, OU_UINT8_BITS - 1, OU_UINT8_BITS);
		
		if (bToggleSecondResult || sfTestFlags.QueryFlagsAllValues() != (uint8ou)(OU_INT8_MIN + 1))
		{
			break;
		}
		
		bool bToggleThirdResult = sfTestFlags.EnumToggleEnumeratedFlagValue(1, 0, OU_UINT8_BITS);
		
		if (!bToggleThirdResult || sfTestFlags.QueryFlagsAllValues() != (uint8ou)OU_INT8_MIN)
		{
			break;
		}
		
		bool bToggleFourthResult = sfTestFlags.EnumToggleEnumeratedFlagValue(1, OU_UINT8_BITS - 1, OU_UINT8_BITS);
		
		if (!bToggleFourthResult || sfTestFlags.QueryFlagsAllValues() != 0)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags8_EnumModifyEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags8 sfTestFlags;
		
		bool bModifyFirstResult = sfTestFlags.EnumModifyEnumeratedFlagValue(1, 0, OU_UINT8_BITS, true);
		
		if (!bModifyFirstResult || sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		bool bModifySecondResult = sfTestFlags.EnumModifyEnumeratedFlagValue(1, OU_UINT8_BITS - 1, OU_UINT8_BITS, true);
		
		if (!bModifySecondResult || sfTestFlags.QueryFlagsAllValues() != (uint8ou)(OU_INT8_MIN + 1))
		{
			break;
		}
		
		bool bModifyThirdResult = sfTestFlags.EnumModifyEnumeratedFlagValue(1, 0, OU_UINT8_BITS, true);
		
		if (bModifyThirdResult || sfTestFlags.QueryFlagsAllValues() != (uint8ou)(OU_INT8_MIN + 1))
		{
			break;
		}
		
		bool bModifyFourthResult = sfTestFlags.EnumModifyEnumeratedFlagValue(1, OU_UINT8_BITS - 1, OU_UINT8_BITS, true);
		
		if (bModifyFourthResult || sfTestFlags.QueryFlagsAllValues() != (uint8ou)(OU_INT8_MIN + 1))
		{
			break;
		}
		
		bool bModifyFifthResult = sfTestFlags.EnumModifyEnumeratedFlagValue(1, 0, OU_UINT8_BITS, false);
		
		if (!bModifyFifthResult || sfTestFlags.QueryFlagsAllValues() != (uint8ou)OU_INT8_MIN)
		{
			break;
		}
		
		bool bModifySixthResult = sfTestFlags.EnumModifyEnumeratedFlagValue(1, OU_UINT8_BITS - 1, OU_UINT8_BITS, false);
		
		if (!bModifySixthResult || sfTestFlags.QueryFlagsAllValues() != 0)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags8_EnumSignalFirstEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags8 sfTestFlags;

		bool bFirstResult = sfTestFlags.EnumSignalFirstEnumeratedFlagValue(1, 0, OU_UINT8_BITS);

		if (!bFirstResult || sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		bool bSecondResult = sfTestFlags.EnumSignalFirstEnumeratedFlagValue(1, 0, OU_UINT8_BITS);
		
		if (bSecondResult || sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		bool bThirdResult = sfTestFlags.EnumSignalFirstEnumeratedFlagValue(2, 0, OU_UINT8_BITS - 1);
		
		if (!bThirdResult || sfTestFlags.QueryFlagsAllValues() != 3)
		{
			break;
		}
		
		bool bFourthResult = sfTestFlags.EnumSignalFirstEnumeratedFlagValue(1, OU_UINT8_BITS - 1, OU_UINT8_BITS);
		
		if (bFourthResult || sfTestFlags.QueryFlagsAllValues() != (uint8ou)(OU_INT8_MIN + 3))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags8_EnumSignalLastEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags8 sfTestFlags;
		
		bool bFirstResult = sfTestFlags.EnumSignalLastEnumeratedFlagValue(1, 0, 1);
		
		if (!bFirstResult || sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		bool bSecondResult = sfTestFlags.EnumSignalLastEnumeratedFlagValue(1, 0, 1);
		
		if (bSecondResult || sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}
		
		bool bThirdResult = sfTestFlags.EnumSignalLastEnumeratedFlagValue(1, 1, 2);
		
		if (!bThirdResult || sfTestFlags.QueryFlagsAllValues() != 3)
		{
			break;
		}
		
		bool bFourthResult = sfTestFlags.EnumSignalLastEnumeratedFlagValue(1, OU_UINT8_BITS - 1, OU_UINT8_BITS);
		
		if (bFourthResult || sfTestFlags.QueryFlagsAllValues() != (uint8ou)(OU_INT8_MIN + 3))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags8_EnumGetEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags8 sfTestFlags((uint8ou)(OU_INT8_MIN + 1));
		
		if (!sfTestFlags.EnumGetEnumeratedFlagValue(1, 0, OU_UINT8_BITS))
		{
			break;
		}
	
		if (sfTestFlags.EnumGetEnumeratedFlagValue(2, 0, OU_UINT8_BITS - 1))
		{
			break;
		}
		
		if (sfTestFlags.EnumGetEnumeratedFlagValue(1, 1, OU_UINT8_BITS - 1))
		{
			break;
		}
		
		if (!sfTestFlags.EnumGetEnumeratedFlagValue(1, OU_UINT8_BITS - 1, OU_UINT8_BITS))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags8_EnumFindFirstEnumeratedFlag()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags8 sfTestFlags((uint8ou)(OU_INT8_MIN + 1));

		unsigned int uiFirstResult = sfTestFlags.EnumFindFirstEnumeratedFlag(1, OU_UINT8_BITS);
		if (uiFirstResult != 0)
		{
			break;
		}

		unsigned int uiSecondResult = sfTestFlags.EnumFindFirstEnumeratedFlag(2, OU_UINT8_BITS - 1);
		if (uiSecondResult != OU_UINT8_BITS - 2)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags8_EnumAllSignalEnumeratedFlags()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags8 sfTestFlags;
		
		sfTestFlags.EnumAllSignalEnumeratedFlags(1, 1);

		if (sfTestFlags.QueryFlagsAllValues() != 1)
		{
			break;
		}

		sfTestFlags.EnumAllSignalEnumeratedFlags(4, OU_UINT8_BITS - 2);

		if (sfTestFlags.QueryFlagsAllValues() != (uint8ou)(OU_UINT8_MAX ^ 2))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags8_EnumAllDropEnumeratedFlags()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags8 sfTestFlags(OU_UINT8_MAX);
		
		sfTestFlags.EnumAllDropEnumeratedFlags(1, 1);
		
		if (sfTestFlags.QueryFlagsAllValues() != (uint8ou)(OU_UINT8_MAX ^ 1))
		{
			break;
		}
		
		sfTestFlags.EnumAllDropEnumeratedFlags(4, OU_UINT8_BITS - 2);
		
		if (sfTestFlags.QueryFlagsAllValues() != 2)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags8_EnumAllQueryEnumeratedFlags()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags8 sfTestFlags((uint8ou)(OU_INT8_MIN + 1));
		
		uint8ou uiFirstResult = sfTestFlags.EnumAllQueryEnumeratedFlags(1, OU_UINT8_BITS);
		if (uiFirstResult != (uint8ou)(OU_INT8_MIN + 1))
		{
			break;
		}
		
		uint8ou uiSecondResult = sfTestFlags.EnumAllQueryEnumeratedFlags(2, OU_UINT8_BITS - 1);
		if (uiSecondResult != (uint8ou)(OU_INT8_MIN))
		{
			break;
		}
		
		uint8ou uiThirdResult = sfTestFlags.EnumAllQueryEnumeratedFlags(2, OU_UINT8_BITS - 2);
		if (uiThirdResult != 0)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags8_EnumAnyGetEnumeratedFlagValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags8 sfTestFlags((uint8ou)(OU_INT8_MIN + 1));
		
		bool bFirstResult = sfTestFlags.EnumAnyGetEnumeratedFlagValue(1, OU_UINT8_BITS);
		if (!bFirstResult)
		{
			break;
		}
		
		bool bSecondResult = sfTestFlags.EnumAnyGetEnumeratedFlagValue(2, OU_UINT8_BITS - 1);
		if (!bSecondResult)
		{
			break;
		}
		
		bool bThirdResult = sfTestFlags.EnumAnyGetEnumeratedFlagValue(2, OU_UINT8_BITS - 2);
		if (bThirdResult)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags8_StoreFlagsEnumeratedValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags8 sfTestFlags;

		sfTestFlags.StoreFlagsEnumeratedValue(0x03, 1, 2);

		if (sfTestFlags.QueryFlagsAllValues() != (uint8ou)(2 << 1))
		{
			break;
		}
	
		sfTestFlags.StoreFlagsEnumeratedValue(0x03, OU_UINT8_BITS - 2, 3);
		
		if (sfTestFlags.QueryFlagsAllValues() != ((uint8ou)(2 << 1) | (uint8ou)(OU_INT8_MIN | (OU_INT8_MIN >> 1))))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestSimpleFlags8_RetrieveFlagsEnumeratedValue()
{
	bool bResult = false;
	
	do
	{
		CSimpleFlags8 sfTestFlags((uint8ou)(OU_INT8_MIN + 1));
		
		unsigned int uiFirstResult = sfTestFlags.RetrieveFlagsEnumeratedValue(0x3, 1);
		if (uiFirstResult != 0)
		{
			break;
		}
		
		unsigned int uiSecondResult = sfTestFlags.RetrieveFlagsEnumeratedValue(0x3, OU_UINT8_BITS - 2);
		if (uiSecondResult != 2)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}


enum EOUSIMPLEFLAGSFEATURE8
{
	OSF8__MIN,
		
	OSF8_CONSTRUCTORS = OSF8__MIN,
	OSF8_ASSIGNFLAGSALLVALUES,
	OSF8_QUERYFLAGSALLVALUES,
	OSF8_SETFLAGSMASKVALUE,
	OSF8_SIGNALFLAGSMASKVALUE,
	OSF8_DROPFLAGSMASKVALUE,
	OSF8_TOGGLESINGLEFLAGVALUE,
	OSF8_MODIFYSINGLEFLAGVALUE,
	OSF8_ASSIGNFLAGSBYMASK,
	OSF8_ALTERFLAGSBYMASK,
	OSF8_GETFLAGSMASKVALUE,
	OSF8_QUERYFLAGSBYMASK,
	OSF8_ONLYSIGNALSINGLEFLAGOUTOFMASK,
	OSF8_ENUMSETENUMERATEDFLAGVALUE,
	OSF8_ENUMSIGNALENUMERATEDFLAGVALUE,
	OSF8_ENUMDROPENUMERATEDFLAGVALUE,
	OSF8_ENUMTOGGLEENUMERATEDFLAGVALUE,
	OSF8_ENUMMODIFYENUMERATEDFLAGVALUE,
	OSF8_ENUMSIGNALFIRSTENUMERATEDFLAGVALUE,
	OSF8_ENUMSIGNALLASTENUMERATEDFLAGVALUE,
	OSF8_ENUMGETENUMERATEDFLAGVALUE,
	OSF8_ENUMFINDFIRSTENUMERATEDFLAG,
	OSF8_ENUMALLSIGNALENUMERATEDFLAGS,
	OSF8_ENUMALLDROPENUMERATEDFLAGS,
	OSF8_ENUMALLQUERYENUMERATEDFLAGS,
	OSF8_ENUMANYGETENUMERATEDFLAGVALUE,
	OSF8_STOREFLAGSENUMERATEDVALUE,
	OSF8_RETRIEVEFLAGSENUMERATEDVALUE,

	OSF8__MAX,
};

template<>
CFeatureTestProcedure const CEnumUnsortedElementArray<EOUSIMPLEFLAGSFEATURE8, OSF8__MAX, CFeatureTestProcedure>::m_aetElementArray[] =
{
	&TestSimpleFlags8_Constructors, // OSF8_CONSTRUCTORS
	&TestSimpleFlags8_AssignFlagsAllValues, // OSF8_ASSIGNFLAGSALLVALUES,
	&TestSimpleFlags8_QueryFlagsAllValues, // OSF8_QUERYFLAGSALLVALUES,
	&TestSimpleFlags8_SetFlagsMaskValue, // OSF8_SETFLAGSMASKVALUE,
	&TestSimpleFlags8_SignalFlagsMaskValue, // OSF8_SIGNALFLAGSMASKVALUE,
	&TestSimpleFlags8_DropFlagsMaskValue, // OSF8_DROPFLAGSMASKVALUE,
	&TestSimpleFlags8_ToggleSingleFlagValue, // OSF8_TOGGLESINGLEFLAGVALUE,
	&TestSimpleFlags8_ModifySingleFlagValue, // OSF8_MODIFYSINGLEFLAGVALUE,
	&TestSimpleFlags8_AssignFlagsByMask, // OSF8_ASSIGNFLAGSBYMASK,
	&TestSimpleFlags8_AlterFlagsByMask, // OSF8_ALTERFLAGSBYMASK,
	&TestSimpleFlags8_GetFlagsMaskValue, // OSF8_GETFLAGSMASKVALUE,
	&TestSimpleFlags8_QueryFlagsByMask, // OSF8_QUERYFLAGSBYMASK,
	&TestSimpleFlags8_OnlySignalSingleFlagOutOfMask, // OSF8_ONLYSIGNALSINGLEFLAGOUTOFMASK,
	&TestSimpleFlags8_EnumSetEnumeratedFlagValue, // OSF8_ENUMSETENUMERATEDFLAGVALUE,
	&TestSimpleFlags8_EnumSignalEnumeratedFlagValue, // OSF8_ENUMSIGNALENUMERATEDFLAGVALUE,
	&TestSimpleFlags8_EnumDropEnumeratedFlagValue, // OSF8_ENUMDROPENUMERATEDFLAGVALUE,
	&TestSimpleFlags8_EnumToggleEnumeratedFlagValue, // OSF8_ENUMTOGGLEENUMERATEDFLAGVALUE,
	&TestSimpleFlags8_EnumModifyEnumeratedFlagValue, // OSF8_ENUMMODIFYENUMERATEDFLAGVALUE,
	&TestSimpleFlags8_EnumSignalFirstEnumeratedFlagValue, // OSF8_ENUMSIGNALFIRSTENUMERATEDFLAGVALUE,
	&TestSimpleFlags8_EnumSignalLastEnumeratedFlagValue, // OSF8_ENUMSIGNALLASTENUMERATEDFLAGVALUE,
	&TestSimpleFlags8_EnumGetEnumeratedFlagValue, // OSF8_ENUMGETENUMERATEDFLAGVALUE,
	&TestSimpleFlags8_EnumFindFirstEnumeratedFlag, // OSF8_ENUMFINDFIRSTENUMERATEDFLAG,
	&TestSimpleFlags8_EnumAllSignalEnumeratedFlags, // OSF8_ENUMALLSIGNALENUMERATEDFLAGS,
	&TestSimpleFlags8_EnumAllDropEnumeratedFlags, // OSF8_ENUMALLDROPENUMERATEDFLAGS,
	&TestSimpleFlags8_EnumAllQueryEnumeratedFlags, // OSF8_ENUMALLQUERYENUMERATEDFLAGS,
	&TestSimpleFlags8_EnumAnyGetEnumeratedFlagValue, // OSF8_ENUMANYGETENUMERATEDFLAGVALUE,
	&TestSimpleFlags8_StoreFlagsEnumeratedValue, // OSF8_STOREFLAGSENUMERATEDVALUE,
	&TestSimpleFlags8_RetrieveFlagsEnumeratedValue, // OSF8_RETRIEVEFLAGSENUMERATEDVALUE,
};
static const CEnumUnsortedElementArray<EOUSIMPLEFLAGSFEATURE8, OSF8__MAX, CFeatureTestProcedure> g_afnSimpleFlags8FeatureTestProcedures;

template<>
const char *const CEnumUnsortedElementArray<EOUSIMPLEFLAGSFEATURE8, OSF8__MAX, const char *>::m_aetElementArray[] =
{
	"Constructors", // OSF8_CONSTRUCTORS
	"AssignFlagsAllValues", // OSF8_ASSIGNFLAGSALLVALUES,
	"QueryFlagsAllValues", // OSF8_QUERYFLAGSALLVALUES,
	"SetFlagsMaskValue", // OSF8_SETFLAGSMASKVALUE,
	"SignalFlagsMaskValue", // OSF8_SIGNALFLAGSMASKVALUE,
	"DropFlagsMaskValue", // OSF8_DROPFLAGSMASKVALUE,
	"ToggleSingleFlagValue", // OSF8_TOGGLESINGLEFLAGVALUE,
	"ModifySingleFlagValue", // OSF8_MODIFYSINGLEFLAGVALUE,
	"AssignFlagsByMask", // OSF8_ASSIGNFLAGSBYMASK,
	"AlterFlagsByMask", // OSF8_ALTERFLAGSBYMASK,
	"GetFlagsMaskValue", // OSF8_GETFLAGSMASKVALUE,
	"QueryFlagsByMask", // OSF8_QUERYFLAGSBYMASK,
	"OnlySignalSingleFlagOutOfMask", // OSF8_ONLYSIGNALSINGLEFLAGOUTOFMASK,
	"EnumSetEnumeratedFlagValue", // OSF8_ENUMSETENUMERATEDFLAGVALUE,
	"EnumSignalEnumeratedFlagValue", // OSF8_ENUMSIGNALENUMERATEDFLAGVALUE,
	"EnumDropEnumeratedFlagValue", // OSF8_ENUMDROPENUMERATEDFLAGVALUE,
	"EnumToggleEnumeratedFlagValue", // OSF8_ENUMTOGGLEENUMERATEDFLAGVALUE,
	"EnumModifyEnumeratedFlagValue", // OSF8_ENUMMODIFYENUMERATEDFLAGVALUE,
	"EnumSignalFirstEnumeratedFlagValue", // OSF8_ENUMSIGNALFIRSTENUMERATEDFLAGVALUE,
	"EnumSignalLastEnumeratedFlagValue", // OSF8_ENUMSIGNALLASTENUMERATEDFLAGVALUE,
	"EnumGetEnumeratedFlagValue", // OSF8_ENUMGETENUMERATEDFLAGVALUE,
	"EnumFindFirstEnumeratedFlag", // OSF8_ENUMFINDFIRSTENUMERATEDFLAG,
	"EnumAllSignalEnumeratedFlags", // OSF8_ENUMALLSIGNALENUMERATEDFLAGS,
	"EnumAllDropEnumeratedFlags", // OSF8_ENUMALLDROPENUMERATEDFLAGS,
	"EnumAllQueryEnumeratedFlags", // OSF8_ENUMALLQUERYENUMERATEDFLAGS,
	"EnumAnyGetEnumeratedFlagValue", // OSF8_ENUMANYGETENUMERATEDFLAGVALUE,
	"StoreFlagsEnumeratedValue", // OSF8_STOREFLAGSENUMERATEDVALUE,
	"RetrieveFlagsEnumeratedValue", // OSF8_RETRIEVEFLAGSENUMERATEDVALUE,
};
static const CEnumUnsortedElementArray<EOUSIMPLEFLAGSFEATURE8, OSF8__MAX, const char *> g_aszSimpleFlags8FeatureTestNames;


bool TestSimpleFlags8(unsigned int &nOutSuccessCount, unsigned int &nOutTestCount)
{
	return TestSubsystem(nOutSuccessCount, nOutTestCount, OSF8__MAX, g_aszSimpleFlags8FeatureTestNames.GetStoragePointer(), g_afnSimpleFlags8FeatureTestProcedures.GetStoragePointer());
}


//////////////////////////////////////////////////////////////////////////

bool TestFlagsDefines_EnumFlagsMask()
{
	bool bResult = false;
	
	do
	{
		int64ou iMask;
		
		iMask = OU_FLAGS_ENUMFLAGS_MASK(uint8ou, 1, 1);
		if (iMask - 1 != 0)
		{
			break;
		}
	
		iMask = OU_FLAGS_ENUMFLAGS_MASK(uint8ou, 1, OU_UINT8_BITS);
		if (iMask ^ OU_UINT8_MAX)
		{
			break;
		}

		iMask = OU_FLAGS_ENUMFLAGS_MASK(uint16ou, 1, 1);
		if (iMask - 1 != 0)
		{
			break;
		}
		
		iMask = OU_FLAGS_ENUMFLAGS_MASK(uint16ou, 1, OU_UINT16_BITS);
		if (iMask ^ OU_UINT16_MAX)
		{
			break;
		}
		
		iMask = OU_FLAGS_ENUMFLAGS_MASK(uint32ou, 1, 1);
		if (iMask - 1 != 0)
		{
			break;
		}
		
		iMask = OU_FLAGS_ENUMFLAGS_MASK(uint32ou, 1, OU_UINT32_BITS);
		if (iMask ^ OU_UINT32_MAX)
		{
			break;
		}
		
		iMask = OU_FLAGS_ENUMFLAGS_MASK(uint64ou, 1, 1);
		if (iMask - 1 != 0)
		{
			break;
		}
		
		iMask = OU_FLAGS_ENUMFLAGS_MASK(uint64ou, 1, OU_UINT64_BITS);
		if (iMask ^ OU_UINT64_MAX)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestFlagsDefines_EnumFlagsStartValid()
{
	bool bResult = false;
	
	do
	{
		if (!OU_FLAGS_ENUMFLAGS_START_VALID(uint8ou, 1, OU_UINT8_BITS))
		{
			break;
		}
/*	
		if (OU_FLAGS_ENUMFLAGS_START_VALID(uint8ou, 1, OU_UINT8_BITS + 1))
		{
			break;
		}
*/		
		if (!OU_FLAGS_ENUMFLAGS_START_VALID(uint16ou, 1, OU_UINT16_BITS))
		{
			break;
		}
/*		
		if (OU_FLAGS_ENUMFLAGS_START_VALID(uint16ou, 1, OU_UINT16_BITS + 1))
		{
			break;
		}
*/		
		if (!OU_FLAGS_ENUMFLAGS_START_VALID(uint32ou, 1, OU_UINT32_BITS))
		{
			break;
		}
/*		
		if (OU_FLAGS_ENUMFLAGS_START_VALID(uint32ou, 1, OU_UINT32_BITS + 1))
		{
			break;
		}
*/		
		if (!OU_FLAGS_ENUMFLAGS_START_VALID(uint64ou, 1, OU_UINT64_BITS))
		{
			break;
		}
/*		
		if (OU_FLAGS_ENUMFLAGS_START_VALID(uint64ou, 1, OU_UINT64_BITS + 1))
		{
			break;
		}
*/		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestFlagsDefines_StoreEnumValueInMask()
{
	bool bResult = false;
	
	do
	{
		if (!OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint8ou, 0, 1) || !OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint8ou, 1, 1))
		{
			break;
		}

		if (!OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint8ou, 0, OU_UINT8_MAX) || !OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint8ou, OU_UINT8_MAX, OU_UINT8_MAX))
		{
			break;
		}

		if (OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint8ou, 1, 2) || OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint8ou, 2, 1))
		{
			break;
		}

		if (OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint8ou, 0, 0) || OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint8ou, 1, 0))
		{
			break;
		}
	
		if (!OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint16ou, 0, 1) || !OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint16ou, 1, 1))
		{
			break;
		}
		
		if (!OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint16ou, 0, OU_UINT16_MAX) || !OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint16ou, OU_UINT16_MAX, OU_UINT16_MAX))
		{
			break;
		}
		
		if (OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint16ou, 1, 2) || OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint16ou, 2, 1))
		{
			break;
		}
		
		if (OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint16ou, 0, 0) || OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint16ou, 1, 0))
		{
			break;
		}
		
		if (!OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint32ou, 0, 1) || !OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint32ou, 1, 1))
		{
			break;
		}
		
		if (!OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint32ou, 0, OU_UINT32_MAX) || !OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint32ou, OU_UINT32_MAX, OU_UINT32_MAX))
		{
			break;
		}
		
		if (OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint32ou, 1, 2) || OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint32ou, 2, 1))
		{
			break;
		}
		
		if (OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint32ou, 0, 0) || OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint32ou, 1, 0))
		{
			break;
		}
		
		if (!OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint64ou, 0, 1) || !OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint64ou, 1, 1))
		{
			break;
		}
		
		if (!OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint64ou, 0, OU_UINT64_MAX) || !OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint64ou, OU_UINT64_MAX, OU_UINT64_MAX))
		{
			break;
		}
		
		if (OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint64ou, 1, 2) || OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint64ou, 2, 1))
		{
			break;
		}
		
		if (OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint64ou, 0, 0) || OU_FLAGS_STOREENUM_VALUE_IN_MASK(uint64ou, 1, 0))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

bool TestFlagsDefines_FlagIsSingle()
{
	bool bResult = false;
	
	do
	{
		if (!OU_FLAGS_FLAG_IS_SINGLE(uint8ou, 1) || !OU_FLAGS_FLAG_IS_SINGLE(uint8ou, OU_INT8_MIN))
		{
			break;
		}

		if (OU_FLAGS_FLAG_IS_SINGLE(uint8ou, 0) || OU_FLAGS_FLAG_IS_SINGLE(uint8ou, 3) || OU_FLAGS_FLAG_IS_SINGLE(uint8ou, OU_INT8_MIN + 1))
		{
			break;
		}
	
		if (!OU_FLAGS_FLAG_IS_SINGLE(uint16ou, 1) || !OU_FLAGS_FLAG_IS_SINGLE(uint16ou, OU_INT16_MIN))
		{
			break;
		}
		
		if (OU_FLAGS_FLAG_IS_SINGLE(uint16ou, 0) || OU_FLAGS_FLAG_IS_SINGLE(uint16ou, 3) || OU_FLAGS_FLAG_IS_SINGLE(uint16ou, OU_INT16_MIN + 1))
		{
			break;
		}
		
		if (!OU_FLAGS_FLAG_IS_SINGLE(uint32ou, 1) || !OU_FLAGS_FLAG_IS_SINGLE(uint32ou, OU_INT32_MIN))
		{
			break;
		}
		
		if (OU_FLAGS_FLAG_IS_SINGLE(uint32ou, 0) || OU_FLAGS_FLAG_IS_SINGLE(uint32ou, 3) || OU_FLAGS_FLAG_IS_SINGLE(uint32ou, OU_INT32_MIN + 1))
		{
			break;
		}
		
		if (!OU_FLAGS_FLAG_IS_SINGLE(uint64ou, 1) || !OU_FLAGS_FLAG_IS_SINGLE(uint64ou, OU_INT64_MIN))
		{
			break;
		}
		
		if (OU_FLAGS_FLAG_IS_SINGLE(uint64ou, 0) || OU_FLAGS_FLAG_IS_SINGLE(uint64ou, 3) || OU_FLAGS_FLAG_IS_SINGLE(uint64ou, OU_INT64_MIN + 1))
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}


enum EOUFLAGSDEFINESFEATURE
{
	OFF__MIN,
	
	OFF_ENUMFLAGS_MASK = OFF__MIN,
	OFF_ENUMFLAGS_START_VALID,
	OFF_STOREENUM_VALUE_IN_MASK,
	OFF_FLAG_IS_SINGLE,
	
	OFF__MAX,
};

template<>
CFeatureTestProcedure const CEnumUnsortedElementArray<EOUFLAGSDEFINESFEATURE, OFF__MAX, CFeatureTestProcedure>::m_aetElementArray[] =
{
	&TestFlagsDefines_EnumFlagsMask, // OFF_ENUMFLAGS_MASK,
	&TestFlagsDefines_EnumFlagsStartValid, // OFF_ENUMFLAGS_START_VALID,
	&TestFlagsDefines_StoreEnumValueInMask, // OFF_STOREENUM_VALUE_IN_MASK,
	&TestFlagsDefines_FlagIsSingle, // OFF_FLAG_IS_SINGLE,
};
static const CEnumUnsortedElementArray<EOUFLAGSDEFINESFEATURE, OFF__MAX, CFeatureTestProcedure> g_afnFlagsDefineFeatureTestProcedures;

template<>const char *const CEnumUnsortedElementArray<EOUFLAGSDEFINESFEATURE, OFF__MAX, const char *>::m_aetElementArray[] =
{
	"ENUMFLAGS_MASK", // OFF_ENUMFLAGS_MASK = OFF__MIN,
	"ENUMFLAGS_START_VALID", // OFF_ENUMFLAGS_START_VALID,
	"STOREENUM_VALUE_IN_MASK", // OFF_STOREENUM_VALUE_IN_MASK,
	"FLAG_IS_SINGLE", // OFF_FLAG_IS_SINGLE,
};
static const CEnumUnsortedElementArray<EOUFLAGSDEFINESFEATURE, OFF__MAX, const char *> g_aszFlagsDefineFeatureTestNames;


bool TestFlagsDefines(unsigned int &nOutSuccessCount, unsigned int &nOutTestCount)
{
	return TestSubsystem(nOutSuccessCount, nOutTestCount, OFF__MAX, g_aszFlagsDefineFeatureTestNames.GetStoragePointer(), g_afnFlagsDefineFeatureTestProcedures.GetStoragePointer());
}


//////////////////////////////////////////////////////////////////////////

enum EENUMARRAYTESTENUM
{
	ATE__MIN,

	ATE_FIRSTELEMENT = ATE__MIN,
	ATE_SECONDELEMENT,
	ATE_THIRDELEMENT,

	ATE__MAX,
};

template<>
const int CEnumUnsortedElementArray<EENUMARRAYTESTENUM, ATE__MAX, int>::m_aetElementArray[] =
{
	1, // ATE_FIRSTELEMENT,
	3, // ATE_SECONDELEMENT,
	2, // ATE_THIRDELEMENT,
};
static const CEnumUnsortedElementArray<EENUMARRAYTESTENUM, ATE__MAX, int> g_ai_IntUnsortedArray;

template<>
const int CEnumSortedElementArray<EENUMARRAYTESTENUM, ATE__MAX, int>::m_aetElementArray[] =
{
	1, // ATE_FIRSTELEMENT,
	2, // ATE_SECONDELEMENT,
	3, // ATE_THIRDELEMENT,
};
static const CEnumSortedElementArray<EENUMARRAYTESTENUM, ATE__MAX, int> g_ai_IntSortedArray;

struct ConstCharPtrEq
{
	bool operator ()(const char *szLeftValue, const char *szRightValue) const
	{
		return strcmp(szLeftValue, szRightValue) == 0;
	}
};

struct ConstCharPtrLess
{
	bool operator ()(const char *szLeftValue, const char *szRightValue) const
	{
		return strcmp(szLeftValue, szRightValue) < 0;
	}
};

template<>
const char *const CEnumUnsortedElementArray<EENUMARRAYTESTENUM, ATE__MAX, const char *, 0, ConstCharPtrEq>::m_aetElementArray[] =
{
	"first",
	"third",
	"second",
};
static const CEnumUnsortedElementArray<EENUMARRAYTESTENUM, ATE__MAX, const char *, 0, ConstCharPtrEq> g_aszStringUnsortedArray;

template<>
const char *const CEnumSortedElementArray<EENUMARRAYTESTENUM, ATE__MAX, const char *, 0, ConstCharPtrLess>::m_aetElementArray[] =
{
	"first",
	"second",
	"third",
};
static const CEnumSortedElementArray<EENUMARRAYTESTENUM, ATE__MAX, const char *, 0, ConstCharPtrLess> g_aszStringSortedArray;


bool TestEnumArrays_UnsortedArray()
{
	EENUMARRAYTESTENUM teEnumCurrent = ATE__MIN;
	
	for (; teEnumCurrent != ATE__MAX; ++teEnumCurrent)
	{
		int iCurrentValue = g_ai_IntUnsortedArray.Encode(teEnumCurrent);
		EENUMARRAYTESTENUM teIntDecodeCheck = g_ai_IntUnsortedArray.Decode(iCurrentValue);
		
		if (!g_ai_IntUnsortedArray.IsValidDecode(teIntDecodeCheck) || teIntDecodeCheck != teEnumCurrent)
		{
			break;
		}

		if (g_ai_IntUnsortedArray.GetStoragePointer()[teEnumCurrent] != iCurrentValue)
		{
			break;
		}

		const char *szCurrentString = g_aszStringUnsortedArray.Encode(teEnumCurrent);
		EENUMARRAYTESTENUM teStringDecodeCheck = g_aszStringUnsortedArray.Decode(szCurrentString);
		
		if (!g_aszStringUnsortedArray.IsValidDecode(teStringDecodeCheck) || teStringDecodeCheck != teEnumCurrent)
		{
			break;
		}

		if (strcmp(g_aszStringUnsortedArray.GetStoragePointer()[teEnumCurrent], szCurrentString) != 0)
		{
			break;
		}

		EENUMARRAYTESTENUM teInvalidDecodeCheck = g_aszStringUnsortedArray.Decode(szCurrentString + 1);
		if (teInvalidDecodeCheck != ATE__MAX || g_aszStringUnsortedArray.IsValidDecode(teInvalidDecodeCheck))
		{
			break;
		}
	}

	bool bResult = teEnumCurrent == ATE__MAX;
	return bResult;
}

bool TestEnumArrays_SortedArray()
{
	EENUMARRAYTESTENUM teEnumCurrent = ATE__MIN;
	
	for (; teEnumCurrent != ATE__MAX; ++teEnumCurrent)
	{
		int iCurrentValue = g_ai_IntSortedArray.Encode(teEnumCurrent);
		EENUMARRAYTESTENUM teIntDecodeCheck = g_ai_IntSortedArray.Decode(iCurrentValue);
		
		if (!g_ai_IntSortedArray.IsValidDecode(teIntDecodeCheck) || teIntDecodeCheck != teEnumCurrent)
		{
			break;
		}

		if (g_ai_IntSortedArray.GetStoragePointer()[teEnumCurrent] != iCurrentValue)
		{
			break;
		}

		const char *szCurrentString = g_aszStringSortedArray.Encode(teEnumCurrent);
		EENUMARRAYTESTENUM teStringDecodeCheck = g_aszStringSortedArray.Decode(szCurrentString);
		
		if (!g_aszStringSortedArray.IsValidDecode(teStringDecodeCheck) || teStringDecodeCheck != teEnumCurrent)
		{
			break;
		}

		if (strcmp(g_aszStringSortedArray.GetStoragePointer()[teEnumCurrent], szCurrentString) != 0)
		{
			break;
		}

		EENUMARRAYTESTENUM teInvalidDecodeCheck = g_aszStringSortedArray.Decode(szCurrentString + 1);
		if (teInvalidDecodeCheck != ATE__MAX || g_aszStringSortedArray.IsValidDecode(teInvalidDecodeCheck))
		{
			break;
		}
	}

	bool bResult = teEnumCurrent == ATE__MAX;
	return bResult;
}


enum EOUENUMARRAYSFEATURE
{
	ORF__MIN,

	ORF_UNSORTEDARRAY = ORF__MIN,
	ORF_SORTEDARRAY,

	ORF__MAX,
};

template<>
CFeatureTestProcedure const CEnumUnsortedElementArray<EOUENUMARRAYSFEATURE, ORF__MAX, CFeatureTestProcedure>::m_aetElementArray[] =
{
	&TestEnumArrays_UnsortedArray, // ORF_UNSORTEDARRAY,
	&TestEnumArrays_SortedArray, // ORF_SORTEDARRAY,
};
static const CEnumUnsortedElementArray<EOUENUMARRAYSFEATURE, ORF__MAX, CFeatureTestProcedure> g_afnEnumArrayFeatureTestProcedures;

template<>
const char *const CEnumUnsortedElementArray<EOUENUMARRAYSFEATURE, ORF__MAX, const char *>::m_aetElementArray[] =
{
	"Unsorted Array", // ORF_UNSORTEDARRAY,
	"Sorted Array", // ORF_SORTEDARRAY,
};
static const CEnumUnsortedElementArray<EOUENUMARRAYSFEATURE, ORF__MAX, const char *> g_aszEnumArrayFeatureTestNames;


bool TestEnumArrays(unsigned int &nOutSuccessCount, unsigned int &nOutTestCount)
{
	return TestSubsystem(nOutSuccessCount, nOutTestCount, ORF__MAX, g_aszEnumArrayFeatureTestNames.GetStoragePointer(), g_afnEnumArrayFeatureTestProcedures.GetStoragePointer());
}


//////////////////////////////////////////////////////////////////////////

enum ETESTTEMPLATES8
{
	TT8_ONE,
	TT8_TWO,
};

enum ETESTTEMPLATES16
{
	TT16_ONE = 1000,
	TT16_TWO,
};

enum ETESTTEMPLATES32
{
	TT32_ONE = 100000,
	TT32_TWO,
};


bool TestTemplates_PrefixIncrement()
{
	bool bResult = false;

	do
	{
		ETESTTEMPLATES8 t8Test = TT8_ONE;
		ETESTTEMPLATES16 t16Test = TT16_ONE;
		ETESTTEMPLATES32 t32Test = TT32_ONE;

		if (++t8Test != TT8_TWO || t8Test != TT8_TWO)
		{
			break;
		}

		++t8Test = TT8_ONE;
		if (t8Test != TT8_ONE)
		{
			break;
		}

		if (++t16Test != TT16_TWO || t16Test != TT16_TWO)
		{
			break;
		}

		++t16Test = TT16_ONE;
		if (t16Test != TT16_ONE)
		{
			break;
		}

		if (++t32Test != TT32_TWO || t32Test != TT32_TWO)
		{
			break;
		}

		++t32Test = TT32_ONE;
		if (t32Test != TT32_ONE)
		{
			break;
		}

		bResult = true;
	}
	while (false);

	return bResult;
}

bool TestTemplates_PostfixIncrement()
{
	bool bResult = false;

	do
	{
		ETESTTEMPLATES8 t8Test = TT8_ONE;
		ETESTTEMPLATES16 t16Test = TT16_ONE;
		ETESTTEMPLATES32 t32Test = TT32_ONE;

		if (t8Test++ != TT8_ONE || t8Test != TT8_TWO)
		{
			break;
		}

		if (t16Test++ != TT16_ONE || t16Test != TT16_TWO)
		{
			break;
		}

		if (t32Test++ != TT32_ONE || t32Test != TT32_TWO)
		{
			break;
		}

		bResult = true;
	}
	while (false);

	return bResult;
}

bool TestTemplates_PrefixDecrement()
{
	bool bResult = false;

	do
	{
		ETESTTEMPLATES8 t8Test = TT8_TWO;
		ETESTTEMPLATES16 t16Test = TT16_TWO;
		ETESTTEMPLATES32 t32Test = TT32_TWO;

		if (--t8Test != TT8_ONE || t8Test != TT8_ONE)
		{
			break;
		}

		--t8Test = TT8_TWO;
		if (t8Test != TT8_TWO)
		{
			break;
		}

		if (--t16Test != TT16_ONE || t16Test != TT16_ONE)
		{
			break;
		}

		--t16Test = TT16_TWO;
		if (t16Test != TT16_TWO)
		{
			break;
		}

		if (--t32Test != TT32_ONE || t32Test != TT32_ONE)
		{
			break;
		}

		--t32Test = TT32_TWO;
		if (t32Test != TT32_TWO)
		{
			break;
		}

		bResult = true;
	}
	while (false);

	return bResult;
}

bool TestTemplates_PostfixDecrement()
{
	bool bResult = false;

	do
	{
		ETESTTEMPLATES8 t8Test = TT8_TWO;
		ETESTTEMPLATES16 t16Test = TT16_TWO;
		ETESTTEMPLATES32 t32Test = TT32_TWO;

		if (t8Test-- != TT8_TWO || t8Test != TT8_ONE)
		{
			break;
		}

		if (t16Test-- != TT16_TWO || t16Test != TT16_ONE)
		{
			break;
		}

		if (t32Test-- != TT32_TWO || t32Test != TT32_ONE)
		{
			break;
		}

		bResult = true;
	}
	while (false);

	return bResult;
}

bool TestTemplates_IsEmptySz()
{
	bool bResult = false;

	do
	{
		const char *szData = "a";
		const char *szEmpty = "";
		const char *szNull = NULL;

		if (IsEmptySz(szData) || !IsEmptySz(szEmpty) || !IsEmptySz(szNull))
		{
			break;
		}

		bResult = true;
	}
	while (false);

	return bResult;
}


enum EOUTEMPLATESFEATURE
{
	OTF__MIN,

	OTF_PREFIXINCREMENT = OTF__MIN,
	OTF_POSTFIXINCREMENT,
	OTF_PREFIXDECREMENT,
	OTF_POSTFIXDECREMENT,
	OTF_ISEMPTYSZ,

	OTF__MAX,
};

template<>
CFeatureTestProcedure const CEnumUnsortedElementArray<EOUTEMPLATESFEATURE, OTF__MAX, CFeatureTestProcedure>::m_aetElementArray[] =
{
	&TestTemplates_PrefixIncrement, // OTF_PREFIXINCREMENT,
	&TestTemplates_PostfixIncrement, // OTF_POSTFIXINCREMENT,
	&TestTemplates_PrefixDecrement, // OTF_PREFIXDECREMENT,
	&TestTemplates_PostfixDecrement, // OTF_POSTFIXDECREMENT,
	&TestTemplates_IsEmptySz, // OTF_ISEMPTYSZ,
};
static const CEnumUnsortedElementArray<EOUTEMPLATESFEATURE, OTF__MAX, CFeatureTestProcedure> g_afnTemplateFeatureTestProcedures;

template<>
const char *const CEnumUnsortedElementArray<EOUTEMPLATESFEATURE, OTF__MAX, const char *>::m_aetElementArray[] =
{
	"Prefix Increment", // OTF_PREFIXINCREMENT,
	"Postfix Increment", // OTF_POSTFIXINCREMENT,
	"Prefix Decrement", // OTF_PREFIXDECREMENT,
	"Postfix Decrement", // OTF_POSTFIXDECREMENT,
	"IsEmptySz", // OTF_ISEMPTYSZ,
};
static const CEnumUnsortedElementArray<EOUTEMPLATESFEATURE, OTF__MAX, const char *> g_aszTemplateFeatureTestNames;


bool TestTemplates(unsigned int &nOutSuccessCount, unsigned int &nOutTestCount)
{
	return TestSubsystem(nOutSuccessCount, nOutTestCount, OTF__MAX, g_aszTemplateFeatureTestNames.GetStoragePointer(), g_afnTemplateFeatureTestProcedures.GetStoragePointer());
}


//////////////////////////////////////////////////////////////////////////

typedef CTypeSimpleWrapper<int> CTestWrapper;

bool TestTypeWrappers_Constructors()
{
	CTestWrapper twEmptyWrapper;
	CTestWrapper twZeroWrapper(0);
	CTestWrapper twCopyWrapper(twZeroWrapper);

	return true;
}

bool TestTypeWrappers_Comparison()
{
	bool bResult = false;

	do
	{
		CTestWrapper twOneWrapper(1);
		CTestWrapper twTwoWrapper(2);

		if (!(twTwoWrapper == twTwoWrapper) || twOneWrapper == twTwoWrapper)
		{
			break;
		}

		if (twTwoWrapper != twTwoWrapper || !(twOneWrapper != twTwoWrapper))
		{
			break;
		}

		if (!(twOneWrapper < twTwoWrapper) || twTwoWrapper < twTwoWrapper || twTwoWrapper < twOneWrapper)
		{
			break;
		}

		if (twOneWrapper > twTwoWrapper || twTwoWrapper > twTwoWrapper || !(twTwoWrapper > twOneWrapper))
		{
			break;
		}

		if (!(twOneWrapper <= twTwoWrapper) || !(twTwoWrapper <= twTwoWrapper) || twTwoWrapper <= twOneWrapper)
		{
			break;
		}

		if (twOneWrapper >= twTwoWrapper || !(twTwoWrapper >= twTwoWrapper) || !(twTwoWrapper >= twOneWrapper))
		{
			break;
		}

		bResult = true;
	}
	while (false);

	return bResult;
}

bool TestTypeWrappers_BoolCasts()
{
	bool bResult = false;

	do
	{
		CTestWrapper twZeroWrapper(0);
		CTestWrapper twOneWrapper(1);
/* -- cast to bool is commented in definition
		if (twZeroWrapper || !(false || twOneWrapper))
		{
			break;
		}
*/
		if (!(!twZeroWrapper) || !twOneWrapper)
		{
			break;
		}

		bResult = true;
	}
	while (false);

	return bResult;
}

bool TestTypeWrappers_Assignment()
{
	bool bResult = false;

	do
	{
		CTestWrapper twZeroWrapper(0);
		CTestWrapper twOneWrapper(1);

		CTestWrapper twTestWrapper;

		CTestWrapper &twFirstAssignmentReference = (twTestWrapper = (CTestWrapper::value_type)1);

		if (twTestWrapper != twOneWrapper || &twFirstAssignmentReference != &twTestWrapper)
		{
			break;
		}

		CTestWrapper &twSecondAssignmentReference = (twTestWrapper = twZeroWrapper);

		if (twTestWrapper != twZeroWrapper || &twSecondAssignmentReference != &twTestWrapper)
		{
			break;
		}

		bResult = true;
	}
	while (false);

	return bResult;
}

bool TestTypeWrappers_DataCast()
{
	bool bResult = false;

	do
	{
		const CTestWrapper twZeroWrapper(0);
		CTestWrapper twOneWrapper(1);

		const CTestWrapper::value_type &wtZeroValue = (const CTestWrapper::value_type &)twZeroWrapper;
		CTestWrapper::value_type &wtOneValue = (CTestWrapper::value_type &)twOneWrapper;

		if (wtZeroValue != 0 || wtOneValue != 1)
		{
			break;
		}

		wtOneValue = 0;

		if (twOneWrapper != twZeroWrapper)
		{
			break;
		}

		bResult = true;
	}
	while (false);

	return bResult;
}

bool TestTypeWrappers_DataComparison()
{
	bool bResult = false;

	do
	{
		const CTestWrapper twZeroWrapper(0);
		const CTestWrapper twOneWrapper(1);

		const CTestWrapper::value_type &wtZeroValue = twZeroWrapper;
		const CTestWrapper::value_type &wtOneValue = twOneWrapper;

		if (!(twZeroWrapper == wtZeroValue) || (twZeroWrapper == wtOneValue))
		{
			break;
		}

		if (!(wtZeroValue == twZeroWrapper) || (wtOneValue == twZeroWrapper))
		{
			break;
		}

		if ((twZeroWrapper != wtZeroValue) || !(twZeroWrapper != wtOneValue))
		{
			break;
		}

		if ((wtZeroValue != twZeroWrapper) || !(wtOneValue != twZeroWrapper))
		{
			break;
		}

		if ((twZeroWrapper < wtZeroValue) || !(twZeroWrapper < wtOneValue) || (twOneWrapper < wtZeroValue))
		{
			break;
		}

		if ((wtZeroValue < twZeroWrapper) || (wtOneValue < twZeroWrapper) || !(wtZeroValue < twOneWrapper))
		{
			break;
		}

		if ((twZeroWrapper > wtZeroValue) || (twZeroWrapper > wtOneValue) || !(twOneWrapper > wtZeroValue))
		{
			break;
		}

		if ((wtZeroValue > twZeroWrapper) || !(wtOneValue > twZeroWrapper) || (wtZeroValue > twOneWrapper))
		{
			break;
		}

		if (!(twZeroWrapper <= wtZeroValue) || !(twZeroWrapper <= wtOneValue) || (twOneWrapper <= wtZeroValue))
		{
			break;
		}

		if (!(wtZeroValue <= twZeroWrapper) || (wtOneValue <= twZeroWrapper) || !(wtZeroValue <= twOneWrapper))
		{
			break;
		}

		if (!(twZeroWrapper >= wtZeroValue) || (twZeroWrapper >= wtOneValue) || !(twOneWrapper >= wtZeroValue))
		{
			break;
		}

		if (!(wtZeroValue >= twZeroWrapper) || !(wtOneValue >= twZeroWrapper) || (wtZeroValue >= twOneWrapper))
		{
			break;
		}

		bResult = true;
	}
	while (false);

	return bResult;
}


enum EOUTYPEWRAPPERFEATURE
{
	OWF__MIN,

	OWF_CONSTRUCTORS = OWF__MIN,
	OWF_COMPARISON,
	OWF_BOOLCASTS,
	OWF_ASSIGNMENT,
	OWF_DATACAST,
	OWF_DATACOMPARISON,

	OWF__MAX,
};

template<>
CFeatureTestProcedure const CEnumUnsortedElementArray<EOUTYPEWRAPPERFEATURE, OWF__MAX, CFeatureTestProcedure>::m_aetElementArray[] =
{
	&TestTypeWrappers_Constructors, // OWF_CONSTRUCTORS,
	&TestTypeWrappers_Comparison, // OWF_COMPARISON,
	&TestTypeWrappers_BoolCasts, // OWF_BOOLCASTS,
	&TestTypeWrappers_Assignment, // OWF_ASSIGNMENT,
	&TestTypeWrappers_DataCast, // OWF_DATACAST,
	&TestTypeWrappers_DataComparison, // OWF_DATACOMPARISON,
};
static const CEnumUnsortedElementArray<EOUTYPEWRAPPERFEATURE, OWF__MAX, CFeatureTestProcedure> g_afnTypeWrapperFeatureTestProcedures;

template<>
const char *const CEnumUnsortedElementArray<EOUTYPEWRAPPERFEATURE, OWF__MAX, const char *>::m_aetElementArray[] =
{
	"Constructors", // OWF_CONSTRUCTORS,
	"Comparison Operators", // OWF_COMPARISON,
	"Boolean Casts", // OWF_BOOLCASTS,
	"Assignment Operators", // OWF_ASSIGNMENT,
	"Data Cast", // OWF_DATACAST,
	"Data Comparisons", // OWF_DATACOMPARISON,
};
static const CEnumUnsortedElementArray<EOUTYPEWRAPPERFEATURE, OWF__MAX, const char *> g_aszTypeWrapperFeatureTestNames;


bool TestTypeWrapper(unsigned int &nOutSuccessCount, unsigned int &nOutTestCount)
{
	return TestSubsystem(nOutSuccessCount, nOutTestCount, OWF__MAX, g_aszTypeWrapperFeatureTestNames.GetStoragePointer(), g_afnTypeWrapperFeatureTestProcedures.GetStoragePointer());
}


//////////////////////////////////////////////////////////////////////////

struct CTestCustomizations_Asserts_FailureInfo
{
	EASSERTIONFAILURESEVERITY m_fsFailureSeverity;
	const char *m_szAssertionExpression;
	const char *m_szAssertionFileName;
	unsigned int m_uiAssertionSourceLine;
};

static const CTestCustomizations_Asserts_FailureInfo g_fiAssertInvalidInfo = { AFS__MAX, NULL, NULL, 0  };
static CTestCustomizations_Asserts_FailureInfo g_fiAssertLastInfo;

void _OU_CONVENTION_CALLBACK TestCustomizations_Asserts_AssertionFailure(EASSERTIONFAILURESEVERITY fsFailureSeverity, 
	const char *szAssertionExpression, const char *szAssertionFileName, unsigned int uiAssertionSourceLine)
{
	g_fiAssertLastInfo.m_fsFailureSeverity = fsFailureSeverity;
	g_fiAssertLastInfo.m_szAssertionExpression = szAssertionExpression;
	g_fiAssertLastInfo.m_szAssertionFileName = szAssertionFileName;
	g_fiAssertLastInfo.m_uiAssertionSourceLine = uiAssertionSourceLine;
}


bool TestCustomizations_Asserts()
{
	bool bResult = false;

	CAssertionFailedProcedure fnAssertOldHandler = CAssertionCheckCustomization::GetAssertFailureCustomHandler();
	CAssertionCheckCustomization::CustomizeAssertionChecks(&TestCustomizations_Asserts_AssertionFailure);

	do
	{
#if !defined(NDEBUG)
		// Only callback invocation is checked here.
		// Availability of functionality depending on preprocessor defines
		// is verified in OST_ASSERT subsystem.

		OU_ASSERT(false); // const unsigned int uiAssertToVerifyLines = 14; -- see further in code

		if (g_fiAssertLastInfo.m_fsFailureSeverity != AFS_ASSERT
			|| g_fiAssertLastInfo.m_szAssertionExpression == NULL
			|| g_fiAssertLastInfo.m_szAssertionFileName == NULL
			|| strcmp(g_fiAssertLastInfo.m_szAssertionFileName, __FILE__) != 0
			|| g_fiAssertLastInfo.m_uiAssertionSourceLine == 0)
		{
			break;
		}

		CTestCustomizations_Asserts_FailureInfo fiAssertFailureInfoSave = g_fiAssertLastInfo;
		g_fiAssertLastInfo = g_fiAssertInvalidInfo;

		OU_VERIFY(false); const unsigned int uiAssertToVerifyLines = 14;

		if (g_fiAssertLastInfo.m_fsFailureSeverity != AFS_ASSERT
			|| g_fiAssertLastInfo.m_szAssertionExpression == NULL
			|| strcmp(g_fiAssertLastInfo.m_szAssertionExpression, fiAssertFailureInfoSave.m_szAssertionExpression) != 0
			|| g_fiAssertLastInfo.m_szAssertionFileName == NULL
			|| strcmp(g_fiAssertLastInfo.m_szAssertionFileName, __FILE__) != 0
			|| g_fiAssertLastInfo.m_uiAssertionSourceLine != fiAssertFailureInfoSave.m_uiAssertionSourceLine + uiAssertToVerifyLines)
		{
			break;
		}

		g_fiAssertLastInfo = g_fiAssertInvalidInfo;


#endif // #if !defined(NDEBUG)

/* -- can't verify OU_CHECK() as it crashes the application on failure
		OU_CHECK(false);

		if (g_fiAssertLastInfo.m_fsFailureSeverity != AFS_CHECK
			|| g_fiAssertLastInfo.m_szAssertionExpression == NULL
			|| g_fiAssertLastInfo.m_szAssertionFileName == NULL
			|| strcmp(g_fiAssertLastInfo.m_szAssertionFileName, __FILE__) != 0
			|| g_fiAssertLastInfo.m_uiAssertionSourceLine == 0)
		{
			break;
		}
*/
		bResult = true;
	}
	while (false);

	CAssertionCheckCustomization::CustomizeAssertionChecks(fnAssertOldHandler);

	return bResult;
}

void *const g_pv_MallocResult = (void *)(size_t)0x12345678;
bool g_bMallocInvocation = false, g_bReallocInvocation = false;
bool g_bFreeInvocation = false, g_bFreeSuccess = false;

void *_OU_CONVENTION_CALLBACK TestCustomizations_MemMgr_Alloc(size_t nBlockSize)
{
	g_bMallocInvocation = true;
	return (void *)((ptrdiff_t)g_pv_MallocResult + nBlockSize);
}

void *_OU_CONVENTION_CALLBACK TestCustomizations_MemMgr_Realloc(void *pv_OldBlock, size_t nBlockNewSize)
{
	g_bReallocInvocation = true;
	return (void *)((ptrdiff_t)pv_OldBlock - nBlockNewSize);
}

void _OU_CONVENTION_CALLBACK TestCustomizations_MemMgr_Free(void *pv_OldBlock)
{
	g_bFreeInvocation = true;
	g_bFreeSuccess = pv_OldBlock == g_pv_MallocResult;
}


bool TestCustomizations_MemMgr()
{
	bool bResult = false;

	CMemoryAllocationProcedure fnAllocationOldProcedure = CMemoryManagerCustomization::GetMemoryAllocationCustomProcedure();
	CMemoryReallocationProcedure fnReallocationOldProcedure = CMemoryManagerCustomization::GetMemoryReallocationCustomProcedure();
	CMemoryDeallocationProcedure fnDeallocationOldProcedure = CMemoryManagerCustomization::GetMemoryDeallocationCustomProcedure();
	CMemoryManagerCustomization::CustomizeMemoryManager(&TestCustomizations_MemMgr_Alloc, &TestCustomizations_MemMgr_Realloc, &TestCustomizations_MemMgr_Free);

	do
	{
		const size_t nBlockSize = 0x1000;

		void *pv_BlockAllocated = AllocateMemoryBlock(nBlockSize);

		if (!g_bMallocInvocation 
			|| pv_BlockAllocated != (void *)((size_t)g_pv_MallocResult + nBlockSize))
		{
			break;
		}

		void *pv_BlockReallocated = ReallocateMemoryBlock(pv_BlockAllocated, 2 * nBlockSize);

		if (!g_bReallocInvocation 
			|| pv_BlockReallocated != (void *)((size_t)g_pv_MallocResult - nBlockSize))
		{
			break;
		}

		FreeMemoryBlock(g_pv_MallocResult);

		if (!g_bFreeInvocation || !g_bFreeSuccess)
		{
			break;
		}

		bResult = true;
	}
	while (false);

	CMemoryManagerCustomization::CustomizeMemoryManager(fnAllocationOldProcedure, fnReallocationOldProcedure, fnDeallocationOldProcedure);

	return bResult;
}


enum EOUCUSTOMIZATIONFEATURE
{
	OCF__MIN,
		
	OCF_ASSERTS = OCF__MIN,
	OCF_MEMMGR,
		
	OCF__MAX,
};

template<>
CFeatureTestProcedure const CEnumUnsortedElementArray<EOUCUSTOMIZATIONFEATURE, OCF__MAX, CFeatureTestProcedure>::m_aetElementArray[] =
{
	&TestCustomizations_Asserts, // OCF_ASSERTS,
	&TestCustomizations_MemMgr, // OCF_MEMMGR,
};
static const CEnumUnsortedElementArray<EOUCUSTOMIZATIONFEATURE, OCF__MAX, CFeatureTestProcedure> g_afnCustomizationFeatureTestProcedures;

template<>
const char *const CEnumUnsortedElementArray<EOUCUSTOMIZATIONFEATURE, OCF__MAX, const char *>::m_aetElementArray[] =
{
	"Asserts", // OCF_ASSERTS,
	"Memory Manager", // OCF_MEMMGR,
};
static const CEnumUnsortedElementArray<EOUCUSTOMIZATIONFEATURE, OCF__MAX, const char *> g_aszCustomizationFeatureTestNames;


bool TestCustomization(unsigned int &nOutSuccessCount, unsigned int &nOutTestCount)
{
	return TestSubsystem(nOutSuccessCount, nOutTestCount, OCF__MAX, g_aszCustomizationFeatureTestNames.GetStoragePointer(), g_afnCustomizationFeatureTestProcedures.GetStoragePointer());
}


//////////////////////////////////////////////////////////////////////////

static const size_t g_nTestMallocBlockInitialSize = 1001;
static const size_t g_nTestMallocBlockNextSize = 65501;
static const uint8ou g_uiTestMallocToken = 0xAA;
static void *g_pv_MemoryBlock = NULL;

bool TestMallocs_Allocate()
{
	bool bResult = false;

	do
	{
		g_pv_MemoryBlock = AllocateMemoryBlock(g_nTestMallocBlockInitialSize);

		if (g_pv_MemoryBlock == NULL || OU_ALIGNED_SIZE((size_t)g_pv_MemoryBlock, _OU_MEMORY_REQUIRED_ALIGNMENT) != (size_t)g_pv_MemoryBlock)
		{
			break;
		}

		*((uint8ou *)g_pv_MemoryBlock + g_nTestMallocBlockInitialSize - 1) = g_uiTestMallocToken;
		
		bResult = true;
	}
	while (false);

	return bResult;
}

bool TestMallocs_Reallocate()
{
	OU_ASSERT(g_nTestMallocBlockNextSize > g_nTestMallocBlockInitialSize);

	bool bResult = false;

	do
	{
		void *pv_OldMemoryBlock = g_pv_MemoryBlock;
		g_pv_MemoryBlock = ReallocateMemoryBlock(pv_OldMemoryBlock, g_nTestMallocBlockNextSize);

		if (g_pv_MemoryBlock == NULL || OU_ALIGNED_SIZE((size_t)g_pv_MemoryBlock, _OU_MEMORY_REQUIRED_ALIGNMENT) != (size_t)g_pv_MemoryBlock)
		{
			break;
		}

		if (*((uint8ou *)g_pv_MemoryBlock + g_nTestMallocBlockInitialSize - 1) != g_uiTestMallocToken)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);

	return bResult;
}

bool TestMallocs_Deallocate()
{
	FreeMemoryBlock(g_pv_MemoryBlock);

	FreeMemoryBlock(NULL); // Free must survive NULL-pointer

	return true;
}


enum EOUMALLOCFEATURE
{
	OLF__MIN,
		
	OLF_ALLOCATE = OLF__MIN,
	OLF_REALLOCATE,
	OLF_DEALLOCATE,
		
	OLF__MAX,
};

template<>
CFeatureTestProcedure const CEnumUnsortedElementArray<EOUMALLOCFEATURE, OLF__MAX, CFeatureTestProcedure>::m_aetElementArray[] =
{
	&TestMallocs_Allocate, // OLF_ALLOCATE,
	&TestMallocs_Reallocate, // OLF_REALLOCATE,
	&TestMallocs_Deallocate, // OLF_DEALLOCATE,
};
static const CEnumUnsortedElementArray<EOUMALLOCFEATURE, OLF__MAX, CFeatureTestProcedure> g_afnMallocFeatureTestProcedures;

template<>
const char *const CEnumUnsortedElementArray<EOUMALLOCFEATURE, OLF__MAX, const char *>::m_aetElementArray[] =
{
	"AllocateMemoryBlock", // OLF_ALLOCATE,
	"ReallocateMemoryBlock", // OLF_REALLOCATE,
	"FreeMemoryBlock", // OLF_DEALLOCATE,
};
static const CEnumUnsortedElementArray<EOUMALLOCFEATURE, OLF__MAX, const char *> g_aszMallocFeatureTestNames;


bool TestMalloc(unsigned int &nOutSuccessCount, unsigned int &nOutTestCount)
{
	return TestSubsystem(nOutSuccessCount, nOutTestCount, OLF__MAX, g_aszMallocFeatureTestNames.GetStoragePointer(), g_afnMallocFeatureTestProcedures.GetStoragePointer());
}


//////////////////////////////////////////////////////////////////////////

bool TestAsserts_FalseFunction(bool &bVarInvocation)
{
	bool bResult = true;
	bVarInvocation = !bVarInvocation;

	bool *pv_Pointer = &bResult;
	if (pv_Pointer)
	{
		bResult = false;
	}

	return bResult;
}

bool TestAsserts_TrueFunction(bool &bVarInvocation)
{
	bool bResult = false;
	bVarInvocation = !bVarInvocation;
	
	bool *pv_Pointer = &bResult;
	if (pv_Pointer)
	{
		bResult = true;
	}
	
	return bResult;
}

bool TestAsserts_Assert()
{
	bool bNDebugInvocation = false, bOrdinaryInvocation = false;

#if defined(NDEBUG)
	
	OU_ASSERT(TestAsserts_FalseFunction(bNDebugInvocation));

	bOrdinaryInvocation = true;

#endif // #if defined(NDEBUG)
	
	OU_ASSERT(TestAsserts_TrueFunction(bOrdinaryInvocation));

	return !bNDebugInvocation && bOrdinaryInvocation;
}

bool TestAsserts_Verify()
{
	bool bNDebugInvocation = false, bOrdinaryInvocation = false;
	
#if defined(NDEBUG)
	
	OU_VERIFY(TestAsserts_FalseFunction(bNDebugInvocation));
	

#else // #if !defined(NDEBUG)

	bNDebugInvocation = true;


#endif // #if !defined(NDEBUG)
	
	OU_VERIFY(TestAsserts_TrueFunction(bOrdinaryInvocation));
	
	return bNDebugInvocation && bOrdinaryInvocation;
}

bool TestAsserts_Check()
{
	bool bOrdinaryInvocation = false;
	
	OU_CHECK(TestAsserts_TrueFunction(bOrdinaryInvocation));
	
	return bOrdinaryInvocation;
}


enum EOUASSERTFEATURE
{
	OEF__MIN,
		
	OEF_ASSERT = OEF__MIN,
	OEF_VERIFY,
	OEF_CHECK,
	
	OEF__MAX,
};

template<>
CFeatureTestProcedure const CEnumUnsortedElementArray<EOUASSERTFEATURE, OEF__MAX, CFeatureTestProcedure>::m_aetElementArray[] =
{
	&TestAsserts_Assert, // OEF_ASSERT,
	&TestAsserts_Verify, // OEF_VERIFY,
	&TestAsserts_Check, // OEF_CHECK,
		
};
static const CEnumUnsortedElementArray<EOUASSERTFEATURE, OEF__MAX, CFeatureTestProcedure> g_afnAssertFeatureTestProcedures;

template<>
const char *const CEnumUnsortedElementArray<EOUASSERTFEATURE, OEF__MAX, const char *>::m_aetElementArray[] =
{
	"OU_ASSERT", // OEF_ASSERT,
	"OU_VERIFY", // OEF_VERIFY,
	"OU_CHECK", // OEF_CHECK,
};
static const CEnumUnsortedElementArray<EOUASSERTFEATURE, OEF__MAX, const char *> g_aszAssertFeatureTestNames;


bool TestAssert(unsigned int &nOutSuccessCount, unsigned int &nOutTestCount)
{
	return TestSubsystem(nOutSuccessCount, nOutTestCount, OEF__MAX, g_aszAssertFeatureTestNames.GetStoragePointer(), g_afnAssertFeatureTestProcedures.GetStoragePointer());
}


//////////////////////////////////////////////////////////////////////////

struct CTestIntTypes_int8
{
	int8ou  m_iPad;
	int8ou	m_iValue;
};

bool TestIntTypes_Int8()
{
	bool bResult = false;
	
	do
	{
		if (sizeof(int8ou) != 1 || offsetof(CTestIntTypes_int8, m_iValue) != 1)
		{
			break;
		}

		if (OU_INT8_BITS != sizeof(int8ou) * OU_BITS_IN_BYTE )
		{
			break;
		}

		if ((int8ou)OU_INT8_MIN == 0 || (int8ou)(OU_INT8_MIN << 1) != 0)
		{
			break;
		}

		if (~OU_INT8_MIN != OU_INT8_MAX || ~OU_INT8_MAX != OU_INT8_MIN)
		{
			break;
		}

		bResult = true;
	}
	while (false);
	
	return bResult;
}

struct CTestIntTypes_uint8
{
	int8ou  m_iPad;
	uint8ou	m_iValue;
};

bool TestIntTypes_UInt8()
{
	bool bResult = false;
	
	do
	{
		if (sizeof(uint8ou) != 1 || offsetof(CTestIntTypes_uint8, m_iValue) != 1)
		{
			break;
		}

		if (OU_UINT8_BITS != sizeof(uint8ou) * OU_BITS_IN_BYTE)
		{
			break;
		}
		
		if (OU_UINT8_MIN != 0)
		{
			break;
		}

		if (!(OU_UINT8_MAX & (uint8ou)1) || (OU_UINT8_MAX >> 1) != (uint8ou)OU_INT8_MAX)
		{
			break;
		}
	
		bResult = true;
	}
	while (false);
	
	return bResult;
}

struct CTestIntTypes_int16
{
	int8ou  m_iPad;
	int16ou	m_iValue;
};

bool TestIntTypes_Int16()
{
	bool bResult = false;
	
	do
	{
		if (sizeof(int16ou) != 2 || offsetof(CTestIntTypes_int16, m_iValue) != 2)
		{
			break;
		}
		
		if (OU_INT16_BITS != sizeof(int16ou) * OU_BITS_IN_BYTE )
		{
			break;
		}
		
		if ((int16ou)OU_INT16_MIN == 0 || (int16ou)(OU_INT16_MIN << 1) != 0)
		{
			break;
		}
		
		if (~OU_INT16_MIN != OU_INT16_MAX || ~OU_INT16_MAX != OU_INT16_MIN)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

struct CTestIntTypes_uint16
{
	int8ou  m_iPad;
	uint16ou	m_iValue;
};

bool TestIntTypes_UInt16()
{
	bool bResult = false;
	
	do
	{
		if (sizeof(uint16ou) != 2 || offsetof(CTestIntTypes_uint16, m_iValue) != 2)
		{
			break;
		}
		
		if (OU_UINT16_BITS != sizeof(uint16ou) * OU_BITS_IN_BYTE)
		{
			break;
		}
		
		if (OU_UINT16_MIN != 0)
		{
			break;
		}
		
		if (!(OU_UINT16_MAX & (uint16ou)1) || (OU_UINT16_MAX >> 1) != (uint16ou)OU_INT16_MAX)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

struct CTestIntTypes_int32
{
	int8ou  m_iPad;
	int32ou	m_iValue;
};

bool TestIntTypes_Int32()
{
	bool bResult = false;
	
	do
	{
		if (sizeof(int32ou) != 4 || offsetof(CTestIntTypes_int32, m_iValue) != 4)
		{
			break;
		}
		
		if (OU_INT32_BITS != sizeof(int32ou) * OU_BITS_IN_BYTE )
		{
			break;
		}
		
		if ((int32ou)OU_INT32_MIN == 0 || (int32ou)(OU_INT32_MIN << 1) != 0)
		{
			break;
		}
		
		if (~OU_INT32_MIN != OU_INT32_MAX || ~OU_INT32_MAX != OU_INT32_MIN)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

struct CTestIntTypes_uint32
{
	int8ou  m_iPad;
	uint32ou	m_iValue;
};

bool TestIntTypes_UInt32()
{
	bool bResult = false;
	
	do
	{
		if (sizeof(uint32ou) != 4 || offsetof(CTestIntTypes_uint32, m_iValue) != 4)
		{
			break;
		}
		
		if (OU_UINT32_BITS != sizeof(uint32ou) * OU_BITS_IN_BYTE)
		{
			break;
		}
		
		if (OU_UINT32_MIN != 0)
		{
			break;
		}
		
		if (!(OU_UINT32_MAX & (uint32ou)1) || (OU_UINT32_MAX >> 1) != (uint32ou)OU_INT32_MAX)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

struct CTestIntTypes_int64
{
	int8ou  m_iPad;
	int64ou	m_iValue;
};

bool TestIntTypes_Int64()
{
	bool bResult = false;
	
	do
	{
		if (sizeof(int64ou) != 8)
		{
			break;
		}
		
#if _OU_TARGET_ARCH == _OU_TARGET_ARCH_X86 && _OU_TARGET_OS != _OU_TARGET_OS_MAC
		
		if (offsetof(CTestIntTypes_int64, m_iValue) != 8)
		{
			break;
		}


#endif

		if (OU_INT64_BITS != sizeof(int64ou) * OU_BITS_IN_BYTE )
		{
			break;
		}
		
		if ((int64ou)OU_INT64_MIN == 0 || (int64ou)(OU_INT64_MIN << 1) != 0)
		{
			break;
		}
		
		if (~OU_INT64_MIN != OU_INT64_MAX || ~OU_INT64_MAX != OU_INT64_MIN)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}

struct CTestIntTypes_uint64
{
	int8ou  m_iPad;
	uint64ou	m_iValue;
};

bool TestIntTypes_UInt64()
{
	bool bResult = false;
	
	do
	{
		if (sizeof(uint64ou) != 8)
		{
			break;
		}
		
#if _OU_TARGET_ARCH == _OU_TARGET_ARCH_X86 && _OU_TARGET_OS != _OU_TARGET_OS_MAC
		
		if (offsetof(CTestIntTypes_uint64, m_iValue) != 8)
		{
			break;
		}


#endif

		if (OU_UINT64_BITS != sizeof(uint64ou) * OU_BITS_IN_BYTE)
		{
			break;
		}
		
		if (OU_UINT64_MIN != 0)
		{
			break;
		}
		
		if (!(OU_UINT64_MAX & (uint64ou)1) || (OU_UINT64_MAX >> 1) != (uint64ou)OU_INT64_MAX)
		{
			break;
		}
		
		bResult = true;
	}
	while (false);
	
	return bResult;
}


enum EOUINTTYPEFEATURE
{
	OIF__MIN,
		
	OIF_INT8 = OIF__MIN,
	OIF_UINT8,
	OIF_INT16,
	OIF_UINT16,
	OIF_INT32,
	OIF_UINT32,
	OIF_INT64,
	OIF_UINT64,
	
	OIF__MAX,
};

template<>
CFeatureTestProcedure const CEnumUnsortedElementArray<EOUINTTYPEFEATURE, OIF__MAX, CFeatureTestProcedure>::m_aetElementArray[] =
{
	&TestIntTypes_Int8, // OIF_INT8,
	&TestIntTypes_UInt8, // OIF_UINT8,
	&TestIntTypes_Int16, // OIF_INT16,
	&TestIntTypes_UInt16, // OIF_UINT16,
	&TestIntTypes_Int32, // OIF_INT32,
	&TestIntTypes_UInt32, // OIF_UINT32,
	&TestIntTypes_Int64, // OIF_INT64,
	&TestIntTypes_UInt64, // OIF_UINT64,

};
static const CEnumUnsortedElementArray<EOUINTTYPEFEATURE, OIF__MAX, CFeatureTestProcedure> g_afnIntTypeFeatureTestProcedures;

template<>
const char *const CEnumUnsortedElementArray<EOUINTTYPEFEATURE, OIF__MAX, const char *>::m_aetElementArray[] =
{
	"int8ou", // OIF_INT8,
	"uint8ou", // OIF_UINT8,
	"int16ou", // OIF_INT16,
	"uint16ou", // OIF_UINT16,
	"int32ou", // OIF_INT32,
	"uint32ou", // OIF_UINT32,
	"int64ou", // OIF_INT64,
	"uint64ou", // OIF_UINT64,
};
static const CEnumUnsortedElementArray<EOUINTTYPEFEATURE, OIF__MAX, const char *> g_aszIntTypeFeatureTestNames;


bool TestIntTypes(unsigned int &nOutSuccessCount, unsigned int &nOutTestCount)
{
	return TestSubsystem(nOutSuccessCount, nOutTestCount, OIF__MAX, g_aszIntTypeFeatureTestNames.GetStoragePointer(), g_afnIntTypeFeatureTestProcedures.GetStoragePointer());
}


//////////////////////////////////////////////////////////////////////////

struct CTestMacros_OffsetStruct 
{
	int8ou		m_i8a;
	int16ou		m_i16a;
	int32ou		m_i32a;
	int64ou		m_i64;
	int32ou		m_i32b;
	int16ou		m_i16b;
	int8ou		m_i8b;
};

bool TestMacros_OffsetOf()
{
	size_t sOffset_i8a = offsetof(CTestMacros_OffsetStruct, m_i8a);
	size_t sOffset_i16a = offsetof(CTestMacros_OffsetStruct, m_i16a);
	size_t sOffset_i32a = offsetof(CTestMacros_OffsetStruct, m_i32a);
	size_t sOffset_i64 = offsetof(CTestMacros_OffsetStruct, m_i64);
	size_t sOffset_i32b = offsetof(CTestMacros_OffsetStruct, m_i32b);
	size_t sOffset_i16b = offsetof(CTestMacros_OffsetStruct, m_i16b);
	size_t sOffset_i8b = offsetof(CTestMacros_OffsetStruct, m_i8b);
	size_t sStructSize = sizeof(CTestMacros_OffsetStruct);

	return true
		&& sOffset_i8a == 0
		&& sOffset_i16a == 2
		&& sOffset_i32a == 4
		&& sOffset_i64 == 8
		&& sOffset_i32b == 16
		&& sOffset_i16b == 20
		&& sOffset_i8b == 22
		&& sStructSize == 24;
}

bool TestMacros_AlignedSize()
{
	return true
		&& OU_ALIGNED_SIZE(0, sizeof(int8ou)) == 0
		&& OU_ALIGNED_SIZE(0, sizeof(int16ou)) == 0
		&& OU_ALIGNED_SIZE(0, sizeof(int32ou)) == 0
		&& OU_ALIGNED_SIZE(0, sizeof(int64ou)) == 0
		&& OU_ALIGNED_SIZE(sizeof(int8ou), sizeof(int8ou)) == sizeof(int8ou)
		&& OU_ALIGNED_SIZE(sizeof(int8ou), sizeof(int16ou)) == sizeof(int16ou)
		&& OU_ALIGNED_SIZE(sizeof(int8ou), sizeof(int32ou)) == sizeof(int32ou)
		&& OU_ALIGNED_SIZE(sizeof(int8ou), sizeof(int64ou)) == sizeof(int64ou)
		&& OU_ALIGNED_SIZE(sizeof(int16ou), sizeof(int8ou)) == sizeof(int16ou)
		&& OU_ALIGNED_SIZE(sizeof(int16ou), sizeof(int16ou)) == sizeof(int16ou)
		&& OU_ALIGNED_SIZE(sizeof(int16ou), sizeof(int32ou)) == sizeof(int32ou)
		&& OU_ALIGNED_SIZE(sizeof(int16ou), sizeof(int64ou)) == sizeof(int64ou)
		&& OU_ALIGNED_SIZE(sizeof(int32ou), sizeof(int8ou)) == sizeof(int32ou)
		&& OU_ALIGNED_SIZE(sizeof(int32ou), sizeof(int16ou)) == sizeof(int32ou)
		&& OU_ALIGNED_SIZE(sizeof(int32ou), sizeof(int32ou)) == sizeof(int32ou)
		&& OU_ALIGNED_SIZE(sizeof(int32ou), sizeof(int64ou)) == sizeof(int64ou)
		&& OU_ALIGNED_SIZE(sizeof(int64ou), sizeof(int8ou)) == sizeof(int64ou)
		&& OU_ALIGNED_SIZE(sizeof(int64ou), sizeof(int16ou)) == sizeof(int64ou)
		&& OU_ALIGNED_SIZE(sizeof(int64ou), sizeof(int32ou)) == sizeof(int64ou)
		&& OU_ALIGNED_SIZE(sizeof(int64ou), sizeof(int64ou)) == sizeof(int64ou);
}

bool TestMacros_ArraySize()
{
	static int m_ai_Array1[1];
	static int m_aai_Array11[1][1];
	static int m_ai_Array2[2];
	static int m_aai_Array21[2][1];
	static int m_aai_Array12[1][2];

	return true
		&& OU_ARRAY_SIZE(m_ai_Array1) == 1
		&& OU_ARRAY_SIZE(m_aai_Array11[0]) == 1
		&& OU_ARRAY_SIZE(m_aai_Array11) == 1
		&& OU_ARRAY_SIZE(m_ai_Array2) == 2
		&& OU_ARRAY_SIZE(m_aai_Array21[0]) == 1
		&& OU_ARRAY_SIZE(m_aai_Array21) == 2
		&& OU_ARRAY_SIZE(m_aai_Array12[0]) == 2
		&& OU_ARRAY_SIZE(m_aai_Array12) == 1;
}

bool TestMacros_InIntRange()
{
	char iZero = 0;
	char iOne = 1;
	char iMinusOne = -1;
	unsigned int uiTen = 10;
	unsigned int uiNotZero = ~0U;

	return true
		&& !OU_IN_INT_RANGE(iZero, 0, 0)
		&& !OU_IN_INT_RANGE(iOne, 0, 0)
		&& !OU_IN_INT_RANGE(iMinusOne, 0, 0)
		&& !OU_IN_INT_RANGE(uiTen, 0, 0)
		&& !OU_IN_INT_RANGE(uiNotZero, 0, 0)

		&& OU_IN_INT_RANGE(iZero, 0, 1)
		&& !OU_IN_INT_RANGE(iOne, 0, 1)
		&& !OU_IN_INT_RANGE(iMinusOne, 0, 1)
		&& !OU_IN_INT_RANGE(uiTen, 0, 1)
		&& !OU_IN_INT_RANGE(uiNotZero, 0, 1)

		&& !OU_IN_INT_RANGE(iZero, 1, 2)
		&& OU_IN_INT_RANGE(iOne, 1, 2)
		&& !OU_IN_INT_RANGE(iMinusOne, 1, 2)
		&& !OU_IN_INT_RANGE(uiTen, 1, 2)
		&& !OU_IN_INT_RANGE(uiNotZero, 1, 2)

		&& OU_IN_INT_RANGE(iZero, -1, 1)
		&& !OU_IN_INT_RANGE(iOne, -1, 1)
		&& OU_IN_INT_RANGE(iMinusOne, -1, 1)
		&& !OU_IN_INT_RANGE(uiTen, -1, 1)
		&& OU_IN_INT_RANGE(uiNotZero, -1, 1)

		&& !OU_IN_INT_RANGE(iZero, 1, -1)
		&& OU_IN_INT_RANGE(iOne, 1, -1)
		&& !OU_IN_INT_RANGE(iMinusOne, 1, -1)
		&& OU_IN_INT_RANGE(uiTen, 1, -1)
		&& !OU_IN_INT_RANGE(uiNotZero, 1, -1);
}

bool TestMacros_InI64Range()
{
	char iZero = 0;
	char iOne = 1;
	char iMinusOne = -1;
	unsigned int uiTen = 10;
	unsigned int uiNotZero = ~0U;
	
	return true
		&& !OU_IN_I64_RANGE(iZero, 0, 0)
		&& !OU_IN_I64_RANGE(iOne, 0, 0)
		&& !OU_IN_I64_RANGE(iMinusOne, 0, 0)
		&& !OU_IN_I64_RANGE(uiTen, 0, 0)
		&& !OU_IN_I64_RANGE(uiNotZero, 0, 0)
		
		&& OU_IN_I64_RANGE(iZero, 0, 1)
		&& !OU_IN_I64_RANGE(iOne, 0, 1)
		&& !OU_IN_I64_RANGE(iMinusOne, 0, 1)
		&& !OU_IN_I64_RANGE(uiTen, 0, 1)
		&& !OU_IN_I64_RANGE(uiNotZero, 0, 1)
		
		&& !OU_IN_I64_RANGE(iZero, 1, 2)
		&& OU_IN_I64_RANGE(iOne, 1, 2)
		&& !OU_IN_I64_RANGE(iMinusOne, 1, 2)
		&& !OU_IN_I64_RANGE(uiTen, 1, 2)
		&& !OU_IN_I64_RANGE(uiNotZero, 1, 2)
		
		&& OU_IN_I64_RANGE(iZero, -1, 1)
		&& !OU_IN_I64_RANGE(iOne, -1, 1)
		&& OU_IN_I64_RANGE(iMinusOne, -1, 1)
		&& !OU_IN_I64_RANGE(uiTen, -1, 1)
		&& !OU_IN_I64_RANGE(uiNotZero, -1, 1)
		
		&& !OU_IN_I64_RANGE(iZero, 1, -1)
		&& OU_IN_I64_RANGE(iOne, 1, -1)
		&& !OU_IN_I64_RANGE(iMinusOne, 1, -1)
		&& OU_IN_I64_RANGE(uiTen, 1, -1)
		&& OU_IN_I64_RANGE(uiNotZero, 1, -1);
}

bool TestMacros_InSizetRange()
{
	char iZero = 0;
	char iOne = 1;
	char iMinusOne = -1;
	unsigned int uiTen = 10;
	unsigned int uiNotZero = ~0U;
	
	return true
		&& !OU_IN_SIZET_RANGE(iZero, 0, 0)
		&& !OU_IN_SIZET_RANGE(iOne, 0, 0)
		&& !OU_IN_SIZET_RANGE(iMinusOne, 0, 0)
		&& !OU_IN_SIZET_RANGE(uiTen, 0, 0)
		&& !OU_IN_SIZET_RANGE(uiNotZero, 0, 0)
		
		&& OU_IN_SIZET_RANGE(iZero, 0, 1)
		&& !OU_IN_SIZET_RANGE(iOne, 0, 1)
		&& !OU_IN_SIZET_RANGE(iMinusOne, 0, 1)
		&& !OU_IN_SIZET_RANGE(uiTen, 0, 1)
		&& !OU_IN_SIZET_RANGE(uiNotZero, 0, 1)
		
		&& !OU_IN_SIZET_RANGE(iZero, 1, 2)
		&& OU_IN_SIZET_RANGE(iOne, 1, 2)
		&& !OU_IN_SIZET_RANGE(iMinusOne, 1, 2)
		&& !OU_IN_SIZET_RANGE(uiTen, 1, 2)
		&& !OU_IN_SIZET_RANGE(uiNotZero, 1, 2)
		
		&& OU_IN_SIZET_RANGE(iZero, -1, 1)
		&& !OU_IN_SIZET_RANGE(iOne, -1, 1)
		&& OU_IN_SIZET_RANGE(iMinusOne, -1, 1)
		&& !OU_IN_SIZET_RANGE(uiTen, -1, 1)
		&& OU_IN_SIZET_RANGE(uiNotZero, -1, 1) == (sizeof(size_t) == sizeof(unsigned int))
		
		&& !OU_IN_SIZET_RANGE(iZero, 1, -1)
		&& OU_IN_SIZET_RANGE(iOne, 1, -1)
		&& !OU_IN_SIZET_RANGE(iMinusOne, 1, -1)
		&& OU_IN_SIZET_RANGE(uiTen, 1, -1)
		&& OU_IN_SIZET_RANGE(uiNotZero, 1, -1) != (sizeof(size_t) == sizeof(unsigned int));
}


enum EOUMACROFEATURE
{
	OMF__MIN,

	OMF_OFFSETOF = OMF__MIN,
	OMF_ALIGNEDSIZE,
	OMF_ARRAYSIZE,
	OMF_ININTRANGE,
	OMF_INI64RANGE,
	OMF_INSIZETRANGE,

	OMF__MAX,
};

template<>
CFeatureTestProcedure const CEnumUnsortedElementArray<EOUMACROFEATURE, OMF__MAX, CFeatureTestProcedure>::m_aetElementArray[] =
{
	&TestMacros_OffsetOf, // OMF_OFFSETOF,
	&TestMacros_AlignedSize, // OMF_ALIGNEDSIZE,
	&TestMacros_ArraySize, // OMF_ARRAYSIZE,
	&TestMacros_InIntRange, // OMF_ININTRANGE,
	&TestMacros_InI64Range, // OMF_INI64RANGE,
	&TestMacros_InSizetRange, // OMF_INSIZETRANGE,
};
static const CEnumUnsortedElementArray<EOUMACROFEATURE, OMF__MAX, CFeatureTestProcedure> g_afnMacroFeatureTestProcedures;

template<>
const char *const CEnumUnsortedElementArray<EOUMACROFEATURE, OMF__MAX, const char *>::m_aetElementArray[] =
{
	"offsetof", // OMF_OFFSETOF,
	"OU_ALIGNED_SIZE", // OMF_ALIGNEDSIZE,
	"OU_ARRAY_SIZE", // OMF_ARRAYSIZE,
	"OU_IN_INT_RANGE", // OMF_ININTRANGE,
	"OU_IN_I64_RANGE", // OMF_INI64RANGE,
	"OU_IN_SIZET_RANGE", // OMF_INSIZETRANGE,
};
static const CEnumUnsortedElementArray<EOUMACROFEATURE, OMF__MAX, const char *> g_aszMacroFeatureTestNames;


bool TestMacros(unsigned int &nOutSuccessCount, unsigned int &nOutTestCount)
{
	return TestSubsystem(nOutSuccessCount, nOutTestCount, OMF__MAX, g_aszMacroFeatureTestNames.GetStoragePointer(), g_afnMacroFeatureTestProcedures.GetStoragePointer());
}


//////////////////////////////////////////////////////////////////////////

// Verifies that target order is not changed
template<>
int const CEnumSortedElementArray<int, _OU_TARGET_OS__MAX - 1, int, 100>::m_aetElementArray[] =
{
	_OU_TARGET_OS_GENUNIX, // _OU_TARGET_OS_GENUNIX
	_OU_TARGET_OS_WINDOWS, // _OU_TARGET_OS_WINDOWS
	_OU_TARGET_OS_QNX, // _OU_TARGET_OS_QNX
	_OU_TARGET_OS_MAC, // _OU_TARGET_OS_MAC
	_OU_TARGET_OS_AIX, // _OU_TARGET_OS_AIX
	_OU_TARGET_OS_SUNOS, // _OU_TARGET_OS_SUNOS
	_OU_TARGET_OS_IOS, // _OU_TARGET_OS_IOS
};
static const CEnumSortedElementArray<int, _OU_TARGET_OS__MAX - 1, int, 100> g_ai_TargetOrderCheck;

template<>
const char *const CEnumUnsortedElementArray<int, _OU_TARGET_OS__MAX - 1, const char *, 100>::m_aetElementArray[] =
{
	"GENERIC UNIX", // _OU_TARGET_OS_GENUNIX
	"WINDOWS", // _OU_TARGET_OS_WINDOWS
	"QNX", // _OU_TARGET_OS_QNX
	"MAC", // _OU_TARGET_OS_MAC
	"AIX", // _OU_TARGET_OS_AIX
	"SunOS", // _OU_TARGET_OS_SUNOS
	"iOS", // _OU_TARGET_OS_IOS
};
static const CEnumUnsortedElementArray<int, _OU_TARGET_OS__MAX - 1, const char *, 100> g_aszOSNames;

// Verifies that bits order is not changed
template<>
int const CEnumSortedElementArray<int, _OU_TARGET_BITS__MAX - 1, int, 101>::m_aetElementArray[] =
{
	_OU_TARGET_BITS_32, // _OU_TARGET_BITS_32
	_OU_TARGET_BITS_64, // _OU_TARGET_BITS_64
};
static const CEnumSortedElementArray<int, _OU_TARGET_BITS__MAX - 1, int, 101> g_ai_BitsOrderCheck;

template<>
const char *const CEnumUnsortedElementArray<int, _OU_TARGET_BITS__MAX - 1, const char *, 101>::m_aetElementArray[] =
{
	"32", // _OU_TARGET_BITS_32
	"64", // _OU_TARGET_BITS_64
};
static const CEnumUnsortedElementArray<int, _OU_TARGET_BITS__MAX - 1, const char *, 101> g_aszBitsNames;

// Verifies that architectures order is not changed
template<>
int const CEnumSortedElementArray<int, _OU_TARGET_ARCH__MAX - 1, int, 102>::m_aetElementArray[] =
{
	_OU_TARGET_ARCH_OTHER, // _OU_TARGET_ARCH_OTHER
	_OU_TARGET_ARCH_X86, // _OU_TARGET_ARCH_X86
	_OU_TARGET_ARCH_IA64, // _OU_TARGET_ARCH_IA64
	_OU_TARGET_ARCH_X64, // _OU_TARGET_ARCH_X64
	_OU_TARGET_ARCH_POWERPC, // _OU_TARGET_ARCH_POWERPC
	_OU_TARGET_ARCH_SPARC, // _OU_TARGET_ARCH_SPARC
	_OU_TARGET_ARCH_ARM, // _OU_TARGET_ARCH_ARM
};
static const CEnumSortedElementArray<int, _OU_TARGET_ARCH__MAX - 1, int, 102> g_ai_ArchitecturesOrderCheck;

template<>
const char *const CEnumUnsortedElementArray<int, _OU_TARGET_ARCH__MAX - 1, const char *, 102>::m_aetElementArray[] =
{
	"OTHER", // _OU_TARGET_ARCH_OTHER
	"x86", // _OU_TARGET_ARCH_X86
	"Itanium", // _OU_TARGET_ARCH_IA64
	"x64", // _OU_TARGET_ARCH_X64
	"PowerPC", // _OU_TARGET_ARCH_POWERPC
	"Sparc", // _OU_TARGET_ARCH_SPARC
	"ARM", // _OU_TARGET_ARCH_ARM
};
static const CEnumUnsortedElementArray<int, _OU_TARGET_ARCH__MAX - 1, const char *, 102> g_aszArchitecturesNames;

// Verifies that compilers order is not changed
template<>
int const CEnumSortedElementArray<int, _OU_COMPILER__MAX - 1, int, 103>::m_aetElementArray[] =
{
	_OU_COMPILER__OTHER, // _OU_COMPILER__OTHER,
	_OU_COMPILER_GCC, // _OU_COMPILER_GCC,
	_OU_COMPILER_MSVC, // _OU_COMPILER_MSVC,
};
static const CEnumSortedElementArray<int, _OU_COMPILER__MAX - 1, int, 103> g_ai_CompilersOrderCheck;

template<>
const char *const CEnumUnsortedElementArray<int, _OU_COMPILER__MAX - 1, const char *, 103>::m_aetElementArray[] =
{
	"UNKNOWN", // _OU_COMPILER__OTHER,
	"GCC", // _OU_COMPILER_GCC,
	"MSVC", // _OU_COMPILER_MSVC,
};
static const CEnumUnsortedElementArray<int, _OU_COMPILER__MAX - 1, const char *, 103> g_aszCompilersNames;

// Verifies that compiler versions order is not changed
template<>
int const CEnumSortedElementArray<int, _OU_COMPILER_VERSION__MAX - 1, int, 104>::m_aetElementArray[] =
{
	_OU_COMPILER_VERSION__OTHER, // _OU_COMPILER_VERSION__OTHER,
	_OU_COMPILER_VERSION_MSVC1998, // _OU_COMPILER_VERSION_MSVC1998,
	_OU_COMPILER_VERSION_GCCLT4, // _OU_COMPILER_VERSION_GCCLT4,
};
static const CEnumSortedElementArray<int, _OU_COMPILER_VERSION__MAX - 1, int, 104> g_ai_CompilersVersionOrderCheck;

template<>
const char *const CEnumUnsortedElementArray<int, _OU_COMPILER_VERSION__MAX - 1, const char *, 104>::m_aetElementArray[] =
{
	"OTHER", // _OU_COMPILER_VERSION__OTHER,
	"MSVC1998", // _OU_COMPILER_VERSION_MSVC1998,
	"GCC LESS THAN 4.0", // _OU_COMPILER_VERSION_GCCLT4,
};
static const CEnumUnsortedElementArray<int, _OU_COMPILER_VERSION__MAX - 1, const char *, 104> g_aszCompilerVersionNames;


#define _TESTPLATFORM_DEFINITION_TEXT(Definition) #Definition
#define TESTPLATFORM_TEFINITION_TEXT(Definition) _TESTPLATFORM_DEFINITION_TEXT(Definition)

bool TestPlatform(unsigned int &nOutSuccessCount, unsigned int &nOutTestCount)
{
	const char *szOSName = g_aszOSNames.Encode(_OU_TARGET_OS - 1);
	const char *szBitsName = g_aszBitsNames.Encode(_OU_TARGET_BITS - 1);
	const char *szArchitectureName = g_aszArchitecturesNames.Encode(_OU_TARGET_ARCH - 1);
	const char *szCompilerName = g_aszCompilersNames.Encode(_OU_COMPILER - 1);
	const char *szCompilerVersion = g_aszCompilerVersionNames.Encode(_OU_COMPILER_VERSION - 1);
	
	printf("Target OS:                   %s\n", szOSName);
	printf("Target Bits:                 %s\n", szBitsName);
	printf("Target Architecture          %s\n", szArchitectureName);
	printf("Compiler Name:               %s\n", szCompilerName);
	printf("Compiler Version:            %s\n", szCompilerVersion);
	printf("Method Convention:           %s\n", TESTPLATFORM_TEFINITION_TEXT(=_OU_CONVENTION_METHOD));
	printf("Function Convention:         %s\n", TESTPLATFORM_TEFINITION_TEXT(=_OU_CONVENTION_API));
	printf("Callback Convention:         %s\n", TESTPLATFORM_TEFINITION_TEXT(=_OU_CONVENTION_CALLBACK));
	printf("Alwaysinline definition:     %s\n", TESTPLATFORM_TEFINITION_TEXT(=_OU_ALWAYSINLINE));
	printf("Inline definition:           %s\n", TESTPLATFORM_TEFINITION_TEXT(=_OU_INLINE));
	
	nOutSuccessCount = 0;
	nOutTestCount = 0;
	return true;
}


//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

enum EOUSUBSYSTEMTEST
{
	OST__MIN,

	OST_INTTYPES = OST__MIN,
	OST_MACROS,
	OST_TEMPLATES,
	OST_TYPEWRAPPER,
	OST_ASSERT,
	OST_MALLOC,
	OST_CUSTOMIZATION,
	OST_ENUMARRAYS,
#if _OU_FEATURE_SET >= _OU_FEATURE_SET_ATOMICS
	OST_ATOMIC,
#endif
	OST_FLAGSDEFINES,
#if _OU_FEATURE_SET >= _OU_FEATURE_SET_ATOMICS
	OST_ATOMICFLAGS,
#endif
	OST_SIMPLEFLAGS64,
	OST_SIMPLEFLAGS32,
	OST_SIMPLEFLAGS16,
	OST_SIMPLEFLAGS8,
#if _OU_FEATURE_SET >= _OU_FEATURE_SET_TLS
	OST_TLS,
#endif
	OST_PLATFORM,

	OST__MAX,
};

typedef bool (*COUSubsystemTestProcedure)(unsigned int &nOutSuccessCount, unsigned int &nOutTestCount);

template<>
COUSubsystemTestProcedure const CEnumUnsortedElementArray<EOUSUBSYSTEMTEST, OST__MAX, COUSubsystemTestProcedure>::m_aetElementArray[] =
{
	&TestIntTypes, // OST_INTTYPES,
	&TestMacros, // OST_MACROS,
	&TestTemplates, // OST_TEMPLATES,
	&TestTypeWrapper, // OST_TYPEWRAPPER,
	&TestAssert, // OST_ASSERT,
	&TestMalloc, // OST_MALLOC,
	&TestCustomization, // OST_CUSTOMIZATION,
	&TestEnumArrays, // OST_ENUMARRAYS,
#if _OU_FEATURE_SET >= _OU_FEATURE_SET_ATOMICS
	&TestAtomic, // OST_ATOMIC,
#endif
	&TestFlagsDefines, // OST_FLAGSDEFINES,
#if _OU_FEATURE_SET >= _OU_FEATURE_SET_ATOMICS
	&TestAtomicFlags, // OST_ATOMICFLAGS,
#endif
	&TestSimpleFlags64, // OST_SIMPLEFLAGS64,
	&TestSimpleFlags32, // OST_SIMPLEFLAGS32,
	&TestSimpleFlags16, // OST_SIMPLEFLAGS16,
	&TestSimpleFlags8, // OST_SIMPLEFLAGS8,
#if _OU_FEATURE_SET >= _OU_FEATURE_SET_TLS
	&TestTLS, // OST_TLS,
#endif
	&TestPlatform, // OST_PLATFORM,
};
static const CEnumUnsortedElementArray<EOUSUBSYSTEMTEST, OST__MAX, COUSubsystemTestProcedure> g_afnOUSubsystemTestProcedures;

template<>
const char *const CEnumUnsortedElementArray<EOUSUBSYSTEMTEST, OST__MAX, const char *>::m_aetElementArray[] =
{
	"IntTypes", // OST_INTTYPES,
	"Macros", // OST_MACROS,
	"Templates", // OST_TEMPLATES,
	"TypeWrapper", // OST_TYPEWRAPPER,
	"Assert", // OST_ASSERT,
	"Malloc", // OST_MALLOC,
	"Customization", // OST_CUSTOMIZATION,
	"EnumArrays", // OST_ENUMARRAYS,
#if _OU_FEATURE_SET >= _OU_FEATURE_SET_ATOMICS
	"Atomic", // OST_ATOMIC,
#endif
	"FlagsDefines", // OST_FLAGSDEFINES,
#if _OU_FEATURE_SET >= _OU_FEATURE_SET_ATOMICS
	"AtomicFlags", // OST_ATOMICFLAGS,
#endif
	"SimpleFlags64", // OST_SIMPLEFLAGS64,
	"SimpleFlags32", // OST_SIMPLEFLAGS32,
	"SimpleFlags16", // OST_SIMPLEFLAGS16,
	"SimpleFlags8", // OST_SIMPLEFLAGS8,
#if _OU_FEATURE_SET >= _OU_FEATURE_SET_TLS
	"TLS", // OST_TLS,
#endif
	"Platform", // OST_PLATFORM,
};
static const CEnumUnsortedElementArray<EOUSUBSYSTEMTEST, OST__MAX, const char *> g_aszOUSubsystemNames;


bool ProcessOUCoverageTests(unsigned int &nOutFailureCount)
{
	unsigned int nSuccessCount = 0;

	for (EOUSUBSYSTEMTEST stSubsystemTest = OST__MIN; stSubsystemTest != OST__MAX; ++stSubsystemTest)
	{
		const char *szSubsystemName = g_aszOUSubsystemNames.Encode(stSubsystemTest);
		printf("\nTesting subsystem \"%s\"\n", szSubsystemName);
		printf("---------------------------------------------------\n");

		unsigned int nSubsysytemSuccessCount = 0, nSubsystemTestCount = 1;

		COUSubsystemTestProcedure fnTestProcedure = g_afnOUSubsystemTestProcedures.Encode(stSubsystemTest);
		if (fnTestProcedure(nSubsysytemSuccessCount, nSubsystemTestCount) && nSubsysytemSuccessCount == nSubsystemTestCount)
		{
			nSuccessCount += 1;
		}

		unsigned int nSubsysytemFailureCount = nSubsystemTestCount - nSubsysytemSuccessCount;
		printf("---------------------------------------------------\n");
		printf("Feature tests failed:                %3u out of %3u\n", nSubsysytemFailureCount, nSubsystemTestCount);
	}

	unsigned int nFailureCount = OST__MAX - nSuccessCount;
	
	printf("\n===================================================\n");
	printf("Subsystem tests failed:              %3u out of %3u\n", nFailureCount, (unsigned int)OST__MAX);

	nOutFailureCount = nFailureCount;
	return nSuccessCount == OST__MAX;
}

int main(int argc, char* argv[])
{
	unsigned int nFailureCount;
	ProcessOUCoverageTests(nFailureCount);

	return nFailureCount;
}

