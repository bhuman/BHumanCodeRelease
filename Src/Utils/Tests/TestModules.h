/**
 * @file Infrastructure/TestModules.h
 *
 * This file declares a series of test modules and representations.
 *
 * Module: REQUIRES -> PROVIDES
 * Ac: -> A
 * Bc: -> B
 * Cc: A -> B
 * Dc: -> C
 * Ec: A,B -> C
 *
 * Am: C -> D
 * Bm: B -> A
 * Cm: A,C -> A,B
 *
 * At: aA -> C
 * Bt: aA,bA -> D
 * Ct: B -> aA
 * Dt: cA -> B
 * Et: aB -> C
 *
 * @author Jan Fiedler
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/Streams/AutoStreamable.h"

// Representations
STREAMABLE(A, {, (char) a, });
STREAMABLE(B, {, (short) a, });
STREAMABLE(C, {, (int) a, });
STREAMABLE(D, {, (float) a, });

STREAMABLE_WITH_BASE(aA, A, {,});
STREAMABLE_WITH_BASE(bA, A, {,});
STREAMABLE_WITH_BASE(cA, D, {,});
STREAMABLE_WITH_BASE(aB, A, {,});

// Cognition
MODULE(Ac,
{,
  PROVIDES(A),
});

class Ac : public AcBase
{
protected:
  void update(A& a) override {}
};

MODULE(Bc,
{,
  PROVIDES(B),
});

class Bc : public BcBase
{
protected:
  void update(B& b) override {}
};

MODULE(Cc,
{,
  REQUIRES(A),
  PROVIDES(B),
});

class Cc : public CcBase
{
protected:
  void update(B& b) override {}
};

MODULE(Dc,
{,
  PROVIDES(C),
});

class Dc : public DcBase
{
protected:
  void update(C& c) override {}
};

MODULE(Ec,
{,
  REQUIRES(A),
  REQUIRES(B),
  PROVIDES(C),
});

class Ec : public EcBase
{
protected:
  void update(C& c) override {}
};

// Motion
MODULE(Am,
{,
  REQUIRES(C),
  PROVIDES(D),
});

class Am : public AmBase
{
protected:
  void update(D& d) override {}
};

MODULE(Bm,
{,
  REQUIRES(B),
  PROVIDES(A),
});

class Bm : public BmBase
{
protected:
  void update(A& a) override {}
};

MODULE(Cm,
{,
  REQUIRES(A),
  REQUIRES(C),
  PROVIDES(A),
  PROVIDES(B),
});

class Cm : public CmBase
{
  void update(A& a) override {}
  void update(B& b) override {}
};

// Other
MODULE(At,
{,
  REQUIRES(aA),
  PROVIDES(C),
});

class At : public AtBase
{
protected:
  void update(C& c) override {}
};

MODULE(Bt,
{,
  REQUIRES(aA),
  REQUIRES(bA),
  PROVIDES(D),
});

class Bt : public BtBase
{
protected:
  void update(D& d) override {}
};

MODULE(Ct,
{,
  REQUIRES(B),
  PROVIDES(aA),
});

class Ct : public CtBase
{
protected:
  void update(aA& aA) override {}
};

MODULE(Dt,
{,
  REQUIRES(cA),
  PROVIDES(B),
});

class Dt : public DtBase
{
protected:
  void update(B& b) override {}
};

MODULE(Et,
{,
  REQUIRES(aB),
  PROVIDES(C),
});

class Et : public EtBase
{
protected:
  void update(C& c) override {}
};
