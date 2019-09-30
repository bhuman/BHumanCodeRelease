/**
 * @file ___FILENAME___
 *
 * This file defines a card that <#...#>
 *
 * @author ___FULLUSERNAME___
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"

CARD(___FILEBASENAME___,
{,
});

class ___FILEBASENAME___ : public ___FILEBASENAME___Base
{
  bool preconditions() const override
  {
    return <#...#>;
  }

  void execute() override
  {
  }
};

MAKE_CARD(___FILEBASENAME___);
