#include "PlayersFieldConverter.h"
#include "Tools/Math/Transformation.h"

void PlayersFieldConverter::update(PlayersFieldPercept& playersFieldPercept)
{
  DECLARE_DEBUG_DRAWING("module:PlayersFieldConverter:jerseyScan", "drawingOnImage");
  playersFieldPercept.players.clear();

  for(const auto& player : thePlayersImagePercept.players)
  {
    const Vector2f correctedCenter = theImageCoordinateSystem.toCorrected(Vector2i(player.realCenterX, player.y2));
    const Vector2f correctedLeft = theImageCoordinateSystem.toCorrected(Vector2i(player.x1FeetOnly, player.y2));
    const Vector2f correctedRight = theImageCoordinateSystem.toCorrected(Vector2i(player.x2FeetOnly, player.y2));

    PlayersFieldPercept::PlayerOnField playerOnField = PlayersFieldPercept::PlayerOnField();

    if(Transformation::imageToRobot(correctedCenter, theCameraMatrix, theCameraInfo, playerOnField.centerOnField) &&
       Transformation::imageToRobot(correctedLeft, theCameraMatrix, theCameraInfo, playerOnField.leftOnField) &&
       Transformation::imageToRobot(correctedRight, theCameraMatrix, theCameraInfo, playerOnField.rightOnField))
    {
      playerOnField.lowerCamera = player.lowerCamera;
      playerOnField.detectedJersey = player.detectedJersey;
      playerOnField.detectedFeet = player.detectedFeet;
      playerOnField.ownTeam = player.ownTeam;
      playerOnField.fallen = player.fallen;

      playerOnField.borderInImage[0] = player.x1;
      playerOnField.borderInImage[1] = player.y2;
      playerOnField.borderInImage[2] = player.x2;
      playerOnField.borderInImage[3] = player.y1;

      if(jerseyDetection)
        detectJersey(player, playerOnField);

      playersFieldPercept.players.push_back(playerOnField);
    }
  }
}

/**
 * This function detects the jersey color independent from the PlayersPerceptor.
 *
 * Returns as first element, if jersey is recognized and as second element,
 * which team is recognized.
 */

void PlayersFieldConverter::detectJersey(const PlayersImagePercept::PlayerInImage& playerInImage, PlayersFieldPercept::PlayerOnField& playerOnField)
{
  // reset values:
  playerOnField.detectedJersey = false;
  playerOnField.ownTeam = false;

  // if player is fallen, no detection:
  if(playerInImage.fallen)
    return;

  // some vars:
  Vector2f fieldPosition = playerOnField.centerOnField;
  int feetWidth = playerInImage.x2FeetOnly - playerInImage.x1FeetOnly;

  Vector2f imagePoint;
  // Transform the 3D-coordinates of the jersey to 2D:
  if(Transformation::robotToImage(Vector3f(fieldPosition[0], fieldPosition[1], (float)jerseyHeight), theCameraMatrix, theCameraInfo, imagePoint))
  {
    // Little approximation if feet are not recognized:
    if(!playerInImage.detectedFeet && theCameraInfo.camera == CameraInfo::upper)
    {
      imagePoint[1] -= (int)((theCameraInfo.width * approxNoFeetTeamCalcRobotWidth - feetWidth) * approxNoFeetTeamCalcInfluence);
    }

    // compensate rolling shutter:
    imagePoint = theImageCoordinateSystem.toCorrectedInverse(imagePoint);

    // calculate the box radius by the given parameter and the distance:
    float r = Geometry::getSizeByDistance(theCameraInfo, 1, fieldPosition.norm());
    int boxRadius = (int)(jerseyScanRadius * r);

    // Define the search border:
    int boxX1 = clip((int)imagePoint[0] - boxRadius, 0, theCameraInfo.width - 1);
    int boxX2 = clip((int)imagePoint[0] + boxRadius, 0, theCameraInfo.width - 1);
    int boxY1 = clip((int)imagePoint[1] - boxRadius, 0, theCameraInfo.height - 1);
    int boxY2 = clip((int)imagePoint[1] + boxRadius, 0, theCameraInfo.height - 1);

    // draw the search border:
    RECTANGLE("module:PlayersFieldConverter:jerseyScan", boxX1, boxY1, boxX2, boxY2, 2, Drawings::solidPen, ColorRGBA::violet);

    // If box is out of image:
    if(boxX1 == boxX2 || boxY1 == boxY2)
      return;

    int ownPixels = 0;
    int opponentPixels = 0;
    int totalPixels = 0;

    FieldColors::Color ownTeamColor =
      theOwnTeamInfo.teamColor == TEAM_BLACK ? FieldColors::black :
      theOwnTeamInfo.teamColor == TEAM_WHITE ? FieldColors::white :
      theOwnTeamInfo.teamColor == TEAM_GREEN ? FieldColors::field :
      FieldColors::none;

    FieldColors::Color opponentTeamColor =
      theOpponentTeamInfo.teamColor == TEAM_BLACK ? FieldColors::black :
      theOpponentTeamInfo.teamColor == TEAM_WHITE ? FieldColors::white :
      theOpponentTeamInfo.teamColor == TEAM_GREEN ? FieldColors::field :
      FieldColors::none;

    // walk through image and count pixels:
    for(int y = boxY1; y <= boxY2; y += jerseyScanGridY)
    {
      for(int x = boxX1; x < boxX2; x += jerseyScanGridX)
      {
        ++totalPixels;

        // if one team is black, white or green, the team membership can be decided just by the FieldColor:
        if(ownTeamColor != FieldColors::none || opponentTeamColor != FieldColors::none)
        {
          if(ownTeamColor == theECImage.colored[y][x])
            ++ownPixels;

          else if(opponentTeamColor == theECImage.colored[y][x])
            ++opponentPixels;
        }
        // if both teams have FieldColors::none, they are colored or gray:
        else
        {
          bool ownTeamIsGray = theOwnTeamInfo.teamColor == TEAM_GRAY;
          bool opponentTeamIsGray = theOpponentTeamInfo.teamColor == TEAM_GRAY;

          // if one team is gray, just decide by saturation value:
          if(ownTeamIsGray || opponentTeamIsGray)
          {
            int& grayTeamsPixels = ownTeamIsGray ? ownPixels : opponentPixels;
            int& coloredTeamsPixels = opponentTeamIsGray ? ownPixels : opponentPixels;

            ColorPixel pixel = theImage[y / 2][x / 2];
            unsigned char h, s, i;
            ColorModelConversions::fromYUVToHSI(pixel.y, pixel.cb, pixel.cr, h, s, i);

            if(s < minSaturationToBeAColor)
              ++grayTeamsPixels;
            else
              ++coloredTeamsPixels;

            continue;
          }
          // if both have saturated colored images, decide by hue value:
          else
          {
            // if current pixel is not FieldColors::none, ignore if wished:
            if(theECImage.colored[y][x] != FieldColors::none && ignoreUncoloredJerseys)
              continue;

            // calculate own hue difference:
            int ownTeamDiff = std::abs(teamColorsHue[theOwnTeamInfo.teamColor] - theECImage.hued[y][x]);
            ownTeamDiff = std::min(ownTeamDiff, std::abs(255 - ownTeamDiff));

            // calculate opponent hue difference:
            int opponentTeamDiff = std::abs(teamColorsHue[theOpponentTeamInfo.teamColor] - theECImage.hued[y][x]);
            opponentTeamDiff = std::min(opponentTeamDiff, std::abs(255 - opponentTeamDiff));

            // if own difference is smaller, it is our pixel, else it is the opponents pixel:
            if(ownTeamDiff < opponentTeamDiff)
              ++ownPixels;
            else
              ++opponentPixels;
          }
        }
      }
    }

    // set the team:
    playerOnField.ownTeam = ownPixels > opponentPixels;

    // if the count of pixels of both teams is different enough and there were enough correct pixels, set as detected:
    playerOnField.detectedJersey = (ownPixels > opponentPixels * jerseyDifFactor || ownPixels * jerseyDifFactor < opponentPixels) &&
                                   ((playerOnField.ownTeam ? ownPixels : opponentPixels) >= totalPixels * minCorrectPixels);

    DRAWTEXT("module:PlayersFieldConverter:jerseyScan", boxX1, boxY2 + 20, 10, ColorRGBA::black, (ownPixels > opponentPixels ? "Own Team" : "Opponent Team") << (playerOnField.detectedJersey ? "!" : "?"));
    DRAWTEXT("module:PlayersFieldConverter:jerseyScan", boxX1, boxY2 + 10, 10, ColorRGBA::black, "Own: " + std::to_string(ownPixels) + " | Opponent: " + std::to_string(opponentPixels) + " | Total: " + std::to_string(totalPixels));
  }
}

MAKE_MODULE(PlayersFieldConverter, perception)
