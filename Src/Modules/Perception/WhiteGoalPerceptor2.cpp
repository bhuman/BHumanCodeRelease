/**
* @file WhiteGoalPerceptor2.cpp
* @author Jonas Stiensmeier
*/

#include "WhiteGoalPerceptor2.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Transformation.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Stopwatch.h"

//Program Structure
void WhiteGoalPerceptor2::update(GoalPercept& goalPercept)
{
  goalPercept.goalPosts.clear();
  possiblePosts.clear();
  possibleGoals.clear();
  sinLUT.clear();
  cosLUT.clear();
  sinLUTsA.clear();
  cosLUTsA.clear();

  DECLARE_DEBUG_DRAWING("module:WhiteGoalPerceptor2:Angle", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:WhiteGoalPerceptor2:CheckBetweenLines", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:WhiteGoalPerceptor2:Edges", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:WhiteGoalPerceptor2:EdgesUnfiltered", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:WhiteGoalPerceptor2:HoughLines", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:WhiteGoalPerceptor2:HoughLinesFiltered", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:WhiteGoalPerceptor2:Noise", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:WhiteGoalPerceptor2:ParameterSpace", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:WhiteGoalPerceptor2:PossiblePosts", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:WhiteGoalPerceptor2:ScanJersey", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:WhiteGoalPerceptor2:ScanRegions", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:WhiteGoalPerceptor2:Validation", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:WhiteGoalPerceptor2:ValidationGoal", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("module:WhiteGoalPerceptor2:Threshold", "drawingOnImage");
  if(!theCameraMatrix.isValid)
    return;

  //ignore lower images for now
  if(theCameraInfo.camera == CameraInfo::lower)
    return;

  angleOffsetInRad = Angle::fromDegrees(angleOffsetInDeg);
  angleInRad = calculateAngle();
  STOPWATCH("WhiteGoalPerceptor2:findPossibleGoalPost") findPossibleGoalPosts();

  if(possiblePosts.size() == 0)
    return;

  STOPWATCH("WhiteGoalPerceptor2:createLUT") createLUT(angleInRad);

  STOPWATCH("WhiteGoalPerceptor2:scanGoalPosts") scanGoalPosts();

  //Validation
  PossibleGoal goal;
  STOPWATCH("WhiteGoalPerceptor2:validateGoal") validateGoal(goal);

  STOPWATCH("WhiteGoalPerceptor2:posting") posting(goalPercept, goal);

  possiblePosts.clear();
}

void WhiteGoalPerceptor2::findPossibleGoalPosts()
{
  const float distanceGoalToFieldBorder = theFieldDimensions.xPosOpponentGoal - theFieldDimensions.xPosOpponentFieldBorder;
  float minDistanceSquared = distanceGoalToFieldBorder * distanceGoalToFieldBorder / 4; // the mindistance should be half the distance between goal and fieldborder
  //initalize lastX with a value which is definitly smaller than the most left pixel in the image
  int lastX = -1;
  for(unsigned int i = 0; i < theScanlineRegionsClipped.scanlines.size(); i++)
  {
    if (theScanlineRegionsClipped.scanlines[i].regions.size() < 2 || static_cast<int>(theScanlineRegionsClipped.scanlines[i].x) <= lastX)
      continue;

    const ScanlineRegions::Region& region = theScanlineRegionsClipped.scanlines[i].regions.back(); //take the region which is the one next to the fieldborder

    if(checkRegion(region, theScanlineRegionsClipped.scanlines[i].x, minDistanceSquared)) // check if the region might belong to a goalpost
    {
      //set height to the lower third of the white region
      int height = (theScanlineRegionsClipped.scanlines[i].regions.back().upper * 2 + theScanlineRegionsClipped.scanlines[i].regions.back().lower) / 3;

      Vector2i estimatedBase(theScanlineRegionsClipped.scanlines[i].x, region.lower);
      Vector2f estimatedPosition;
      if(!Transformation::imageToRobotWithCameraRotation(estimatedBase, theCameraMatrix, theCameraInfo, estimatedPosition))
        continue;
      int estimatedRadius = static_cast<int>(Geometry::getSizeByDistance(theCameraInfo, theFieldDimensions.goalPostRadius, estimatedPosition.norm()));

      bool stopSearch = false;

      //scanLeft
      int left = theScanlineRegionsClipped.scanlines[i].x;
      int leftSkipped = 0;
      while(left >= 0 && leftSkipped < 2)
      {
        if(!theColorTable[theImage[height][left]].is(ColorClasses::white))
          leftSkipped++;
        else
          leftSkipped = 0;
        left--;

        if(static_cast<int>(theScanlineRegionsClipped.scanlines[i].x) - left >  4 * estimatedRadius)
        {
          stopSearch = true;
          break;
        }
      }

      if(stopSearch)
        continue;

      //scanRight
      int right = theScanlineRegionsClipped.scanlines[i].x;
      int rightSkipped = 0;
      while(right < theImage.width && rightSkipped < 2)
      {
        if(!theColorTable[theImage[height][right]].is(ColorClasses::white))
          rightSkipped++;
        else
          rightSkipped = 0;
        right++;
        if(static_cast<int>(right - theScanlineRegionsClipped.scanlines[i].x) > 4 * estimatedRadius)
        {
          stopSearch = true;
          break;
        }
      }

      if(stopSearch)
      {
        lastX = right;
        continue;
      }
      int width = right - rightSkipped - left + leftSkipped;
      int mid = left + width / 2;

      //check if the region isn't the lowest part of the post
      int lowest = region.lower - 1;
      int lowerSkipped = 0;
      while(lowest < theImage.height && lowerSkipped < 2)
      {
        if(!theColorTable[theImage[lowest][mid]].is(ColorClasses::white))
          lowerSkipped++;
        else
          lowerSkipped = 0;
        lowest++;
      }
      lowest -= lowerSkipped;

      Vector2i base(mid, lowest);
      Vector2f position;
      if(!Transformation::imageToRobotWithCameraRotation(base, theCameraMatrix, theCameraInfo, position))
        continue;
      PossiblePost post(base);
      createGoalModel(position.norm(), post);
      post.scanAreaSize = static_cast<int>(post.model.radius * scanAreaSizeOffset);

      CROSS("module:WhiteGoalPerceptor2:PossiblePosts", post.base.x(), post.base.y(), 2, 2, Drawings::solidPen, ColorRGBA::black);
      // check if width is between diameter * minWidthOffset (between 0 and 1) and diameter * maxWidthoffset( greater equal 1)
      //and if the goalposts conists of at least 4 Pixel in diameter
      if(post.model.radius >= minRadiusOffset && width >= post.model.radius * 2 * minWidthOffset && width <= post.model.radius * 2 * maxWidthOffset && left >= lastX && isGreenInSurrounding(post.base.x(), post.base.y() + 2))
      {
        LINE("module:WhiteGoalPerceptor2:Angle", base.x(), base.y(), base.x() - sin(angleInRad) * post.model.outerHeight,
             base.y() - cos(angleInRad) * post.model.outerHeight, 1, Drawings::solidPen, ColorRGBA::black);
        //no overlapping of edgemaps
        lastX = right - rightSkipped;
        CROSS("module:WhiteGoalPerceptor2:PossiblePosts", post.base.x(), post.base.y(), 2, 2, Drawings::solidPen, ColorRGBA::blue);
        CROSS("module:WhiteGoalPerceptor2:PossiblePosts", left + leftSkipped, height, 2, 2, Drawings::solidPen, ColorRGBA::green);
        CROSS("module:WhiteGoalPerceptor2:PossiblePosts", right - rightSkipped, height, 2, 2, Drawings::solidPen, ColorRGBA::red);
        possiblePosts.push_back(post);
      }
    }
  }
}

bool WhiteGoalPerceptor2::checkRegion(const ScanlineRegions::Region& region, const int& x, const float& minDistanceSquared)
{
  Vector2f upperOnField;
  Vector2f lowerOnField;
  return region.color.is(ColorClasses::white) &&
    region.lower - region.upper >= 2 &&
    //check if the region is high enough to belong to a goalpost
    Transformation::imageToRobot(Vector2i(x, region.upper), theCameraMatrix, theCameraInfo, upperOnField) &&
    Transformation::imageToRobot(Vector2i(x, region.lower), theCameraMatrix, theCameraInfo, lowerOnField) &&
    (upperOnField - lowerOnField).squaredNorm() >= minDistanceSquared;
}

void WhiteGoalPerceptor2::scanGoalPosts()
{
  //sobelAndHough
  for(auto& possiblePost : possiblePosts)
  {
    const int height = possiblePost.model.outerHeight; // +2 * possiblePost.scanAreaSize;

    possiblePost.imageOrigin = Vector2i(std::max(possiblePost.base.x() - possiblePost.model.radius - possiblePost.scanAreaSize, 0),
                                            std::max(possiblePost.base.y() - height - 1, 0));
    possiblePost.crossBarPosition = possiblePost.base.y() - possiblePost.model.outerHeight;

    //if x percent of the height are not in the image stop the evaluation
    float heightDiff = static_cast<float>(possiblePost.base.y()) / height;
    if(heightDiff < minimalHeightThreshold)
      continue;

    //since it is possible that a goalpost reaches out of the image clip it at the upper and lower side
    possiblePost.imageHeight = std::min(height, possiblePost.base.y() - 1);

    // clip at right side
    possiblePost.imageWidth = std::min((possiblePost.model.radius + possiblePost.scanAreaSize) * 2, theImage.width - possiblePost.imageOrigin.x() - 1);

    //calculate Expected Houghvalue should be outerHeight but should be clipped add imageborder
    //also some pixel will probaby never be detected as edges they can be ignored by setting houghignore
    int houghIgnorePost = static_cast<int>(possiblePost.imageHeight * houghIgnorePostOffset);
    possiblePost.expectedHoughValue = possiblePost.model.outerHeight - houghIgnorePost;

    ASSERT(possiblePost.imageOrigin.x() >= 0 && possiblePost.imageOrigin.y() >= 0 &&
          possiblePost.imageOrigin.x() < theImage.width && possiblePost.imageOrigin.y() < theImage.height);

    ASSERT(possiblePost.imageHeight > 0 && possiblePost.imageWidth > 0);

    STOPWATCH("WhiteGoalPerceptor2:createAccumulatorArray") createAccumulatorArray(possiblePost);

    STOPWATCH("WhiteGoalPerceptor2:calculateEdgeMap") calculateEdgeMap(possiblePost); // 10 ms
    if(!possiblePost.edgeMapSuccess)
      continue;

    if(useHough)
    {
      STOPWATCH("WhiteGoalPerceptor2:fillAccumulatorArray") fillAccumulatorArray(possiblePost); // 17 ms
      STOPWATCH("WhiteGoalPerceptor2:evaluateAccumulatorArray") evaluateAccumulatorArray(possiblePost);
    }
  }
}

void WhiteGoalPerceptor2::validateGoal(PossibleGoal& goal)
{
  for(auto it = possiblePosts.begin(); it != possiblePosts.end();)
  {
    //if no lines have been found remove the post
    if(it->lines.size() == 0)
    {
      it = possiblePosts.erase(it);
    }
    else
    {
      validateGoalPost(*it);
      if(it->value < validationGoalPostThreshold)
        it = possiblePosts.erase(it);
      else
        it++;
    }
  }
  if(possiblePosts.size() == 0)
    return;
  if(possiblePosts.size() > 1)
  {
    ASSERT(distanceValidationOffset >= 1);
    if(possiblePosts.size() > 2)
      possiblePosts.sort([&](const PossiblePost p1, const PossiblePost p2)
                         {
                           return p1.value > p2.value;
                         });

    for(auto i = possiblePosts.begin(); i != possiblePosts.end(); ++i)
    {
      auto j = i;
      for(++j; j != possiblePosts.end(); ++j)
      {
        goal.value = calculateCompleteGoalValue(*i, *j);
        if(goal.value >= validationThreshold)
        {
          goal.posts.push_back(&*i);
          goal.posts.push_back(&*j);
          return;
        }
      }
    }

    if(goal.value < validationThreshold)
    {
      auto& first = *possiblePosts.begin();
      auto& second = *++possiblePosts.begin();
      if(first.value > second.value)
      {
        goal.posts.push_back(&first);
        goal.value = first.value;
      }
      else
      {
        goal.posts.push_back(&second);
        goal.value = second.value;
      }
    }
  }
  else
  {
    goal.value = possiblePosts.front().value;
    goal.posts.push_back(&possiblePosts.front());
  }

  COMPLEX_DRAWING("module:WhiteGoalPerceptor2:ValidationGoal")
  {
    int heightOffset = 0;
    DRAWTEXT("module:WhiteGoalPerceptor2:ValidationGoal", goal.posts[0]->base.x() + goal.posts[0]->model.radius, goal.posts[0]->base.y(), 7, ColorRGBA::black, "Validation: " << goal.value);
    //DRAWTEXT("module:WhiteGoalPerceptor2:HoughLines", 0, -12, 7, ColorRGBA::black, "max Houghvalue: " << max);

    for(PossiblePost* post : goal.posts)
    {
      DRAWTEXT("module:WhiteGoalPerceptor2:ValidationGoal", post->base.x() + post->model.radius, post->base.y() - heightOffset * 8, 7, ColorRGBA::black, "ValidationPost: " << post->value);
      heightOffset--;
    }
  }
}

void WhiteGoalPerceptor2::posting(GoalPercept& goalPercept, PossibleGoal& goal)
{
  if(setPosts(goal))
  {
    if(goal.value > validationThreshold)
    {
      GoalPost post1;
      post1.positionInImage = goal.baseInImageFirstPost;
      post1.positionOnField = goal.baseOnFieldFirstPost;

      if(goal.posts.size() > 1) //Evaluate the second goalpost
      {
        GoalPost post2;
        post2.positionInImage = goal.baseInImageSecondPost;
        post2.positionOnField = goal.baseOnFieldSecondPost;

        post1.position = GoalPost::Position::IS_LEFT;
        post2.position = GoalPost::Position::IS_RIGHT;

        ASSERT(!std::isnan(post2.positionOnField.x()));
        ASSERT(!std::isnan(post2.positionOnField.y()));
        goalPercept.goalPosts.push_back(post2);

        goalPercept.timeWhenCompleteGoalLastSeen = theFrameInfo.time;
      }
      else
      {
        post1.position = GoalPost::Position::IS_UNKNOWN;
        goalPercept.timeWhenGoalPostLastSeen = theFrameInfo.time;
      }

      ASSERT(!std::isnan(post1.positionOnField.x()));
      ASSERT(!std::isnan(post1.positionOnField.y()));
      goalPercept.goalPosts.push_back(post1);
    }
  }
}

//Auxilary Methods
float WhiteGoalPerceptor2::calculateCompleteGoalValue(const PossiblePost& post1, const PossiblePost& post2)
{
  int relativePosition = post1.base.x() - post2.base.x();
  int distance = std::abs(relativePosition);
  int estimatedDistanceOfTwoGoalPosts = (post1.model.innerWidth + post2.model.innerWidth) / 2;

  int baseX = (post1.imageOrigin.x() + post2.imageOrigin.x()) / 2;
  int baseY = post2.imageOrigin.y() + post2.model.radius * 2;
  PossiblePost bar(Vector2i(baseX, baseY));
  bar.isCrossBar = true;
  if(distance > estimatedDistanceOfTwoGoalPosts / 2)
  {
    if(relativePosition < 0)
      scanCrossBar(post1, post2, bar);
    else
      scanCrossBar(post2, post1, bar);
  }

  //initial value = average value of both goalposts
  float value = (post1.value + post2.value) / 2;

  float distanceEvaluation = 1.f;
  float parallelismEvaluation = 1.f;

  float max = std::max(distance, estimatedDistanceOfTwoGoalPosts) * distanceValidationOffset;
  distanceEvaluation = 1 - std::abs(distance - estimatedDistanceOfTwoGoalPosts) / max;

  const float maxAngleDiff = angleOffsetInDeg * 2 * angleValidationOffset;

  //since lines and possibleposts are already sorted
  int diff = std::abs(post1.angleInDeg - post2.angleInDeg);
  parallelismEvaluation = 1 - (diff / maxAngleDiff);

  value *= distanceEvaluation * parallelismEvaluation * (1 + bar.value);
  return value;
}

bool WhiteGoalPerceptor2::setPosts(PossibleGoal& goal)
{
  if(goal.posts.size() == 0)
    return false;

  if(goal.posts.size() == 1)
  {
    goal.baseInImageFirstPost = goal.posts[0]->base;
    if(!transformImageToField(goal.posts[0]->base, goal.baseOnFieldFirstPost))
      return false;
  }
  else
  {
    if(goal.posts[0]->base.x() < goal.posts[1]->base.x())
    {
      goal.baseInImageFirstPost = goal.posts[0]->base;
      goal.baseInImageSecondPost = goal.posts[1]->base;
    }
    else
    {
      goal.baseInImageFirstPost = goal.posts[1]->base;
      goal.baseInImageSecondPost = goal.posts[0]->base;
    }
    if(!transformImageToField(goal.baseInImageFirstPost, goal.baseOnFieldFirstPost))
      return false;
    if(!transformImageToField(goal.baseInImageSecondPost, goal.baseOnFieldSecondPost))
      return false;
  }

  return true;
}

bool WhiteGoalPerceptor2::transformImageToField(Vector2i& positionInImage, Vector2f& positionOnField)
{
  Vector2f corrected = theImageCoordinateSystem.toCorrected(positionInImage);
  return Transformation::imageToRobot(corrected, theCameraMatrix, theCameraInfo, positionOnField);
}

void WhiteGoalPerceptor2::calculateEdgeMap(PossiblePost& post)
{
  post.edgeMap.resize(post.imageHeight);
  int valueB;
  unsigned char value;
  post.edgeMapSuccess = sobel(post);
  if (useNMS)
    nonMaximumSuppression(post);
  if (useOtsu)
    otsuThreshold(post);
  COMPLEX_DRAWING("module:WhiteGoalPerceptor2:Edges")
  {

    if(post.edgeMapSuccess)
    {
      for(int y = 0; y < post.imageHeight; y++)
        for(int x = 0; x < post.imageWidth; x++)
        {
          valueB = post.edgeMap[y][x];
          value = 0;
          if((useOtsu && valueB > post.threshold) ||
             (!useOtsu && valueB > sobelThreshold))
          {
            value = 255;
          }

          DOT("module:WhiteGoalPerceptor2:Edges", post.imageOrigin.x() + x, post.imageOrigin.y() + y, ColorRGBA(value, value, value), ColorRGBA(value, value, value));
        }
    }

    //Draw a black picture margin
    for(int y = 0; y < post.imageHeight; y++)
    {
      DOT("module:WhiteGoalPerceptor2:Edges", post.imageOrigin.x() - 1, post.imageOrigin.y() + y, ColorRGBA(0, 0, 0), ColorRGBA(0, 0, 0));
      DOT("module:WhiteGoalPerceptor2:Edges", post.imageOrigin.x() + post.imageWidth, post.imageOrigin.y() + y, ColorRGBA(0, 0, 0), ColorRGBA(0, 0, 0));
    }
    for(int x = 0; x < post.imageWidth; x++)
    {
      DOT("module:WhiteGoalPerceptor2:Edges", post.imageOrigin.x() + x, post.imageOrigin.y() - 1, ColorRGBA(0, 0, 0), ColorRGBA(0, 0, 0));
      DOT("module:WhiteGoalPerceptor2:Edges", post.imageOrigin.x() + x, post.imageOrigin.y() + post.imageHeight, ColorRGBA(0, 0, 0), ColorRGBA(0, 0, 0));
    }
  }

  COMPLEX_DRAWING("module:WhiteGoalPerceptor2:EdgesUnfiltered")
  {
    if(post.edgeMapSuccess)
    {
      for(int y = 0; y < post.imageHeight; y++)
      {
        for(int x = 0; x < post.imageWidth; x++)
        {
          valueB = post.edgeMap[y][x];

          value = std::min(static_cast<unsigned char>(255), static_cast<unsigned char>(std::sqrt(valueB) * brightenEdgeOffset));

          DOT("module:WhiteGoalPerceptor2:EdgesUnfiltered", post.imageOrigin.x() + x, post.imageOrigin.y() + y, ColorRGBA(value, value, value), ColorRGBA(value, value, value));
        }
      }
    }
    for(int y = 0; y < post.imageHeight; y++)
    {
      DOT("module:WhiteGoalPerceptor2:EdgesUnfiltered", post.imageOrigin.x() - 1, post.imageOrigin.y() + y, ColorRGBA(0, 0, 0), ColorRGBA(0, 0, 0));
      DOT("module:WhiteGoalPerceptor2:EdgesUnfiltered", post.imageOrigin.x() + post.imageWidth, post.imageOrigin.y() + y, ColorRGBA(0, 0, 0), ColorRGBA(0, 0, 0));
    }
    for(int x = 0; x < post.imageWidth; x++)
    {
      DOT("module:WhiteGoalPerceptor2:EdgesUnfiltered", post.imageOrigin.x() + x, post.imageOrigin.y() - 1, ColorRGBA(0, 0, 0), ColorRGBA(0, 0, 0));
      DOT("module:WhiteGoalPerceptor2:EdgesUnfiltered", post.imageOrigin.x() + x, post.imageOrigin.y() + post.imageHeight, ColorRGBA(0, 0, 0), ColorRGBA(0, 0, 0));
    }
  }
}

void WhiteGoalPerceptor2::fillAccumulatorArray(PossiblePost& post)
{
  //fill acumulator array
  for(int y = 0; y < post.imageHeight; y++)
  {
    for(int x = 0; x < post.imageWidth; x++)
    {
      if ((useOtsu && post.edgeMap[y][x] > post.threshold) ||
        (!useOtsu && post.edgeMap[y][x] > sobelThreshold))
      {
        addPixelForHough(post, x, y);
      }
    }
  }

  COMPLEX_DRAWING("module:WhiteGoalPerceptor2:ParameterSpace")
  {
    for(unsigned int y = 0; y < post.houghImg.size(); y++)
    {
      for(unsigned int x = 0; x < post.houghImg[y].size(); x++)
      {
        unsigned char value = static_cast<unsigned char>(post.houghImg[y][x] * parameterSpaceOffset);
        DOT("module:WhiteGoalPerceptor2:ParameterSpace", x + post.imageOrigin.x(), y + -25, ColorRGBA(value, value, value), ColorRGBA(value, value, value));
      }
    }
  }
}

void WhiteGoalPerceptor2::evaluateAccumulatorArray(PossiblePost& post)
{
  int imageCenterY = post.imageHeight / 2;
  int imageCenterX = post.imageWidth / 2;
  int max = 0;
  for(int a = 0; static_cast<unsigned int>(a) < post.houghImg.size(); a++)
  {
    for(int r = 0; static_cast<unsigned int>(r) < post.houghImg[0].size(); r++)
    {
      if(max < post.houghImg[a][r])
        max = post.houghImg[a][r];

      ASSERT(post.expectedHoughValue > 0);

      float value = static_cast<float>(post.houghImg[a][r]) / static_cast<float>(post.expectedHoughValue);
      if(value > 1 - houghOffset && value < 1 + houghOffset)
      {
        Line line;

        line.angleInDeg = a - angleOffsetInDeg + static_cast<int>(post.model.angleInDeg);

        //transformation to 90 degree is done by using the other representation
        if(post.isCrossBar)
        {
          line.x0 = 0;
          line.x1 = post.imageWidth;
          line.y0 = calculateHoughLineForBar(line.x0 - imageCenterX, a, r - post.rMax, imageCenterY);
          line.y1 = calculateHoughLineForBar(line.x1 - imageCenterX, a, r - post.rMax, imageCenterY);
        }
        else
        {
          line.y0 = 0;
          line.y1 = post.imageHeight;
          line.x0 = calculateHoughLineForPost(line.y0 - imageCenterY, a, r - post.rMax, imageCenterX);
          line.x1 = calculateHoughLineForPost(line.y1 - imageCenterY, a, r - post.rMax, imageCenterX);
        }

        line.r = r - post.rMax;

        line.value = post.houghImg[a][r];

        post.lines.push_back(line);
      }
    }
  }

  COMPLEX_DRAWING("module:WhiteGoalPerceptor2:HoughLines")
  {
    if(post.edgeMapSuccess)
    {
      DRAWTEXT("module:WhiteGoalPerceptor2:HoughLines", 0, -12, 7, ColorRGBA::black, "max Houghvalue: " << max);
      for(Line line : post.lines)
      {
        int x0 = line.x0 + post.imageOrigin.x();
        int x1 = line.x1 + post.imageOrigin.x();
        int y0 = line.y0 + post.imageOrigin.y();
        int y1 = line.y1 + post.imageOrigin.y();
        LINE("module:WhiteGoalPerceptor2:HoughLines", x0, y0, x1, y1, 1, Drawings::solidPen, ColorRGBA::blue);
      }
    }
  }
}

void WhiteGoalPerceptor2::validateGoalPost(PossiblePost& post)
{
  if(!post.edgeMapSuccess || post.lines.size() == 0)
    return;
  int x0;
  int x1;
  int y0;
  int y1;
  if(post.lines.size() > 1)
  {
    if (!useAlternativeLineSearch)
      std::sort(post.lines.begin(), post.lines.end(), [&](const Line l1, const Line l2) { return std::abs(l1.value - post.expectedHoughValue) < std::abs(l2.value - post.expectedHoughValue); });

    bool foundAlternativeLine = true;
    unsigned int firstLine = 0;

    int distanceStart;
    int distanceEnd;
    if (useAlternativeLineSearch)
    {
      for (; firstLine < post.lines.size() - 1; firstLine++)
      {
        if (std::abs(post.lines[firstLine].x0 - post.lines[firstLine + 1].x0) > post.model.radius && std::abs(post.lines[firstLine].x1 - post.lines[firstLine + 1].x1) > post.model.radius)
          break;
      }
    }

    int secondLine = firstLine;
    bool crossed = false;
    do
    {
      if (secondLine < static_cast<int>(post.lines.size()) - 1)
        secondLine++;
      else
      {
        foundAlternativeLine = false;
        break;
      }
      if(post.isCrossBar)
      {
        distanceStart = std::abs(post.lines[firstLine].y0 - post.lines[secondLine].y0);
        distanceEnd = std::abs(post.lines[firstLine].y1 - post.lines[secondLine].y1);
      }
      else
      {
        distanceStart = std::abs(post.lines[firstLine].x0 - post.lines[secondLine].x0);
        distanceEnd = std::abs(post.lines[firstLine].x1 - post.lines[secondLine].x1);
        crossed = (post.lines[firstLine].x0 > post.lines[secondLine].x0 && post.lines[firstLine].x1 < post.lines[secondLine].x1) ||
          (post.lines[firstLine].x0 < post.lines[secondLine].x0 && post.lines[firstLine].x1 > post.lines[secondLine].x1);
      }
    }
    while(crossed || distanceStart < post.model.radius * radiusNMSOffset || distanceEnd < post.model.radius * radiusNMSOffset);

    if(foundAlternativeLine)
    {
      float angleDiffCheck = 1.f;
      float angleCheck = 1.f;
      float widthCheck = 1.f;

      // formula for both validations
      //1 - (d / (n * o)), where 1 is best validation, d ist the distance n is for nomalizing it to 0..1 and o is an offset
      //greater than 1

      int diff = std::abs(post.lines[firstLine].angleInDeg - post.lines[secondLine].angleInDeg);
      const float maxAngleDiff = angleOffsetInDeg * 2 * angleValidationOffset;
      angleDiffCheck = 1 - (diff / maxAngleDiff);
      ASSERT(angleDiffCheck >= 0 && angleDiffCheck <= 1);

      post.angleInDeg = (post.lines[firstLine].angleInDeg + post.lines[secondLine].angleInDeg) / 2;
      angleCheck = 1 - std::abs((post.model.angleInDeg - post.angleInDeg)) / maxAngleDiff;
      ASSERT(angleCheck >= 0 && angleCheck <= 1);

      int width = (std::abs(post.lines[firstLine].x1 - post.lines[secondLine].x1) + std::abs(post.lines[firstLine].x0 - post.lines[secondLine].x0)) / 2;
      int expectedWidth;
      if(post.isCrossBar)
        expectedWidth = post.model.innerWidth;
      else
        expectedWidth = post.model.radius * 2;

      widthCheck = 1 - std::abs(width - expectedWidth) / (std::max(width, expectedWidth) * widthValidationOffset);
      ASSERT(widthCheck >= 0 && widthCheck <= 1);

      //not used right now
      float heightCheck = 1.f;
      int heightMax = static_cast<int>(std::max(post.expectedHoughValue, post.lines[firstLine].value) * heightValidationOffset);
      heightCheck = 1.f - static_cast<float>(std::abs(post.expectedHoughValue - post.lines[firstLine].value)) / heightMax;
      ASSERT(heightCheck >= 0 && heightCheck <= 1);

      post.value = angleDiffCheck * widthCheck * angleCheck;
      ASSERT(post.value >= 0 && post.value <= 1);

      //if it might be a goalpost check if there are no edges in the middle of the post
      if(useCheckBetweenLines && post.value > validationGoalPostThreshold && checkBetweenLines(post.lines[firstLine], post.lines[secondLine], width, post))
          post.value = 0;

      if(useNoise && post.value > validationGoalPostThreshold && calculateNoise(post.lines[firstLine], post.lines[secondLine], width, post))
        post.value = 0;

      if(useLinesAsBase)
        post.base.x() = (post.lines[firstLine].x1 + post.lines[secondLine].x1) / 2 + post.imageOrigin.x();

      COMPLEX_DRAWING("module:WhiteGoalPerceptor2:HoughLinesFiltered")
      {
        x0 = post.lines[firstLine].x0 + post.imageOrigin.x();
        x1 = post.lines[firstLine].x1 + post.imageOrigin.x();
        y0 = post.lines[firstLine].y0 + post.imageOrigin.y();
        y1 = post.lines[firstLine].y1 + post.imageOrigin.y();
        LINE("module:WhiteGoalPerceptor2:HoughLinesFiltered", x0, y0, x1, y1, 1, Drawings::solidPen, ColorRGBA::red);
        x0 = post.lines[secondLine].x0 + post.imageOrigin.x();
        x1 = post.lines[secondLine].x1 + post.imageOrigin.x();
        y0 = post.lines[secondLine].y0 + post.imageOrigin.y();
        y1 = post.lines[secondLine].y1 + post.imageOrigin.y();
        LINE("module:WhiteGoalPerceptor2:HoughLinesFiltered", x0, y0, x1, y1, 1, Drawings::solidPen, ColorRGBA::red);
      }
      DRAWTEXT("module:WhiteGoalPerceptor2:Validation", post.base.x() + post.model.radius, post.base.y() - 32, 7, ColorRGBA::black, "height: " << heightCheck);
      DRAWTEXT("module:WhiteGoalPerceptor2:Validation", post.base.x() + post.model.radius, post.base.y() - 24, 7, ColorRGBA::black, "AngleDiff: " << angleDiffCheck);
      DRAWTEXT("module:WhiteGoalPerceptor2:Validation", post.base.x() + post.model.radius, post.base.y() - 16, 7, ColorRGBA::black, "width: " << widthCheck);
      DRAWTEXT("module:WhiteGoalPerceptor2:Validation", post.base.x() + post.model.radius, post.base.y() - 8, 7, ColorRGBA::black, "Angle: " << angleCheck);
      DRAWTEXT("module:WhiteGoalPerceptor2:Validation", post.base.x() + post.model.radius, post.base.y(), 7, ColorRGBA::black, "Validation: " << post.value);
      //to stop the execution of the method at this point a return is used
      return;
    }
  }

  COMPLEX_DRAWING("module:WhiteGoalPerceptor2:HoughLinesFiltered")
  {
    x0 = post.lines[0].x0 + post.imageOrigin.x();
    x1 = post.lines[0].x1 + post.imageOrigin.x();
    y0 = post.lines[0].y0 + post.imageOrigin.y();
    y1 = post.lines[0].y1 + post.imageOrigin.y();
    LINE("module:WhiteGoalPerceptor2:HoughLinesFiltered", x0, y0, x1, y1, 1, Drawings::solidPen, ColorRGBA::yellow);
  }
}

inline int WhiteGoalPerceptor2::calculateHoughLineForPost(const int& y, const int& alpha, const int& r, const int& offset)
{
  //x = (r - y sin(alpha)) / cos(alpha);
  //sinLut[i] = sin(i + minAngle) same for cos
  return static_cast<int>((r - y * sinLUT[alpha]) / cosLUT[alpha]) + offset;
}

inline int WhiteGoalPerceptor2::calculateHoughLineForBar(const int& x, const int& alpha, const int& r, const int& offset)
{
  //y = (r - x * cos(alpha)) / sin(alpha)
  //sinLutsA[i] = sin(i + minAngle + 90degree) same for cos
  return static_cast<int>((r - x * cosLUTsA[alpha]) / sinLUTsA[alpha]) + offset;
}

bool WhiteGoalPerceptor2::checkBetweenLines(const Line& firstLine, const Line& secondLine, const int& width, const PossiblePost& post)
{
  int y = post.imageHeight / 2;
  int counter = 0;
  int numOfPixels = 0;
  for(int t = static_cast<int>(-post.model.innerHeight / 4); t < static_cast<int>(post.model.innerHeight / 4); t++)
  {
    int x1mid = (firstLine.x1 + firstLine.x0) / 2;
    int x0mid = (secondLine.x1 + secondLine.x0) / 2;
    int x = (x1mid + x0mid) / 2;
    x += (firstLine.x1 - firstLine.x0) * (y + t) / post.imageHeight;
    for(int j = std::min(0, static_cast<int>(-width / 4)) + 1; j < std::max(1, static_cast<int>(width / 4)); j++)
    {
      ColorRGBA color = ColorRGBA::green;
      if(y + t >= 0 && y + t < post.imageHeight && x + j >= 0 && x + j  < post.imageWidth &&
        ((useOtsu && post.edgeMap[y + t][x + j] > post.threshold) ||
        (!useOtsu && post.edgeMap[y + t][x + j] > sobelThreshold) ||
         !theColorTable[theImage[y + t + post.imageOrigin.y()][x + j + post.imageOrigin.x()]].is(ColorClasses::white)))
      {
        counter++;
        color = ColorRGBA::red;
      }
      numOfPixels++;
      DOT("module:WhiteGoalPerceptor2:CheckBetweenLines", x + j + post.imageOrigin.x(), y + t + post.imageOrigin.y(), color, color);
    }
  }

  return numOfPixels == 0 || counter > checkBetweenLinesOffset * numOfPixels;
}

bool WhiteGoalPerceptor2::calculateNoise(const Line& firstLine, const Line& secondLine, const int& width, const PossiblePost& post)
{
  const float filter[3][3] = {{1.f, -2.f, 1.f}, {-2.f, 4.f, -2.f}, {1.f, -2.f, 1.f}};
  float sumY = 0.f;
  float sumCb = 0.f;
  float sumCr = 0.f;

  int y = post.imageHeight / 2;
  int numOfPixels = 0;
  for(int t = static_cast<int>(-post.model.innerHeight / 4); t < static_cast<int>(post.model.innerHeight / 4); t++)
  {
    int x1mid = (firstLine.x1 + firstLine.x0) / 2;
    int x0mid = (secondLine.x1 + secondLine.x0) / 2;
    int x = (x1mid + x0mid) / 2;
    x += (firstLine.x1 - firstLine.x0) * (y + t) / post.imageHeight;
    float varianceY = 0.f;
    float varianceCr = 0.f;
    float varianceCb = 0.f;
    for(int i = -1; i <= 1; i++)
    {
      for(int j = -1; j <= 1; j++)
      {
        varianceY += static_cast<int>(filter[i + 1][j + 1] * theImage[y + i + t + post.imageOrigin.y()][x + j + post.imageOrigin.x()].y);
        varianceCb += static_cast<int>(filter[i + 1][j + 1] * theImage[y + i + t + post.imageOrigin.y()][x + j + post.imageOrigin.x()].cb);
        varianceCr += static_cast<int>(filter[i + 1][j + 1] * theImage[y + i + t + post.imageOrigin.y()][x + j + post.imageOrigin.x()].cr);
      }
    }
    sumY += std::abs(varianceY);
    sumCr += std::abs(varianceCr);
    sumCb += std::abs(varianceCb);
    numOfPixels++;
  }

  const float sigmaY = pi_2 * (1.f / (6 * (theImage.width - 2)* (theImage.height - 2))) * sumY;
  const float sigmaCr = pi_2 * (1.f / (6 * (theImage.width - 2)* (theImage.height - 2))) * sumCr;
  const float sigmaCb = pi_2 * (1.f / (6 * (theImage.width - 2)* (theImage.height - 2))) * sumCb;
  const float sigma = (sigmaCb + sigmaCr + sigmaY) * 1000/numOfPixels;

  if(sigma > sigmaThreshold)
  {
    DOT("module:WhiteGoalPerceptor2:Noise", post.base.x(), post.base.y(), ColorRGBA::red, ColorRGBA::red);
  }
  else
  {
    DOT("module:WhiteGoalPerceptor2:Noise", post.base.x(), post.base.y(), ColorRGBA::green, ColorRGBA::green);
  }
  DRAWTEXT("module:WhiteGoalPerceptor2:Noise", post.base.x() + post.model.radius, post.base.y(), 7, ColorRGBA::black, "sigma: " << sigma);
  return sigma > sigmaThreshold;
}

void WhiteGoalPerceptor2::createAccumulatorArray(PossiblePost& post)
{
  //minAngle and maxAngle expected for a "vertical" goalpost
  post.minAngle = static_cast<int>(post.model.angleInDeg - angleOffsetInDeg);
  int maxAngle = static_cast<int>(post.model.angleInDeg + angleOffsetInDeg);
  //numOfAngles
  int numAngle = maxAngle - post.minAngle;

  //center of the Image
  int centerX = post.imageWidth / 2;
  int centerY = post.imageHeight / 2;

  //maximum radius
  post.rMax = static_cast<int>(std::sqrt(centerX*centerX + centerY*centerY));
  //number of radius
  int numR = 2 * post.rMax;

  //create Accumulator Array
  ASSERT(post.houghImg.empty());
  post.houghImg.resize(numAngle);
  for(int i = 0; i < numAngle; i++)
    post.houghImg[i].resize(numR);
}

void WhiteGoalPerceptor2::addPixelForHough(PossiblePost& post, const int& x, const int& y)
{
  int u = x - post.imageWidth / 2;
  int v = y - post.imageHeight / 2;
  int r;

  for(unsigned int alpha = 0; alpha < post.houghImg.size(); alpha++)
  {
    //add rMax to set it between 0 and rMax * 2 instead of -rMax and +rMax
    //sinLutsA[i] = sin(i + minAngle - angleOffset) same for cos
    //r = x * cos(i) + y * sin(i) + offsetX
    if(post.isCrossBar)
      r = static_cast<int>(u * cosLUTsA[alpha] + v * sinLUTsA[alpha]) + post.rMax;
    else
      r = static_cast<int>(u * cosLUT[alpha] + v * sinLUT[alpha]) + post.rMax;
    if(r > 0 && static_cast<unsigned int>(r) < post.houghImg[alpha].size())
      post.houghImg[alpha][r]++;
  }
}

void WhiteGoalPerceptor2::scanCrossBar(const PossiblePost& leftPost, const PossiblePost& rightPost, PossiblePost& bar)
{
  //assume the crossbar is also a post since it is similar
  int scanAreaSize = 2;
  //use the old model and tilt it by 90 degrees compared to a goalpost
  bar.modelCreated = true;
  if(leftPost.modelCreated)
    bar.model = leftPost.model;
  else if(rightPost.modelCreated)
    bar.model = rightPost.model;
  else
    bar.modelCreated = false;

  if(bar.modelCreated)
    bar.model.angleInDeg += 90;

  int baseY;
  //search for a good base of the goalpost in vertical orientation
  if(leftPost.crossBarPosition >= 0 && rightPost.crossBarPosition >= 0)
    baseY = leftPost.crossBarPosition < rightPost.crossBarPosition ? leftPost.crossBarPosition : rightPost.crossBarPosition;
  else if(leftPost.crossBarPosition)
    baseY = leftPost.crossBarPosition;
  else if(rightPost.crossBarPosition)
    baseY = rightPost.crossBarPosition;
  else
    return;

  bar.imageOrigin = Vector2i(std::max(leftPost.base.x() - scanAreaSize - leftPost.model.radius, 0), std::max(baseY - scanAreaSize, 0));
  bar.imageHeight = std::min(bar.model.radius * 2 + scanAreaSize * 2, theImage.height - bar.imageOrigin.y() - 1);
  bar.imageWidth = std::min(rightPost.base.x() - leftPost.base.x() + (bar.model.radius + scanAreaSize) * 2, theImage.width - bar.imageOrigin.x() - 1);

  //calculate Expected Houghvalue should be maximum rightPostbase - leftPostbase but should be clipped add imageborder
  //also some pixel will probaby never be detected as edges they can be ignored by setting houhignore
  int houghIgnoreBar = static_cast<int>(bar.imageHeight * houghIgnoreBarOffset);
  bar.expectedHoughValue = bar.imageWidth - houghIgnoreBar;
  ASSERT(bar.imageOrigin.x() >= 0 && bar.imageOrigin.y() >= 0 && bar.imageOrigin.x() < theImage.width && bar.imageOrigin.y() < theImage.height);

  bar.edgeMap.resize(bar.imageHeight);
  int valueB;
  unsigned char value;

  //sobelAndHough
  createAccumulatorArray(bar);
  calculateEdgeMap(bar);
  if(!bar.edgeMapSuccess)
    return;
  if(useHough)
  {
    fillAccumulatorArray(bar);
    evaluateAccumulatorArray(bar);
  }

  COMPLEX_DRAWING("module:WhiteGoalPerceptor2:Edges")
  {
    if(bar.edgeMapSuccess)
    {
      for(int y = 0; y < bar.imageHeight; y++)
      {
        for(int x = 0; x < bar.imageWidth; x++)
        {
          valueB = bar.edgeMap[y][x];
          value = 0;
          if(valueB > sobelCrossBarThreshold)
          {
            value = 255;
          }

          DOT("module:WhiteGoalPerceptor2:Edges", bar.imageOrigin.x() + x, bar.imageOrigin.y() + y, ColorRGBA(value, value, value), ColorRGBA(value, value, value));
        }
      }
    }

    //Draw a black picture margin
    for(int y = 0; y < bar.imageHeight; y++)
    {
      DOT("module:WhiteGoalPerceptor2:Edges", bar.imageOrigin.x() - 1, bar.imageOrigin.y() + y, ColorRGBA(0, 0, 0), ColorRGBA(0, 0, 0));
      DOT("module:WhiteGoalPerceptor2:Edges", bar.imageOrigin.x() + bar.imageWidth, bar.imageOrigin.y() + y, ColorRGBA(0, 0, 0), ColorRGBA(0, 0, 0));
    }
    for(int x = 0; x < bar.imageWidth; x++)
    {
      DOT("module:WhiteGoalPerceptor2:Edges", bar.imageOrigin.x() + x, bar.imageOrigin.y() - 1, ColorRGBA(0, 0, 0), ColorRGBA(0, 0, 0));
      DOT("module:WhiteGoalPerceptor2:Edges", bar.imageOrigin.x() + x, bar.imageOrigin.y() + bar.imageHeight, ColorRGBA(0, 0, 0), ColorRGBA(0, 0, 0));
    }
  }

  COMPLEX_DRAWING("module:WhiteGoalPerceptor2:EdgesUnfiltered")
  {
    if(bar.edgeMapSuccess)
    {
      for(int y = 0; y < bar.imageHeight; y++)
      {
        for(int x = 0; x < bar.imageWidth; x++)
        {
          valueB = bar.edgeMap[y][x];
          value = std::min(static_cast<unsigned char>(255), static_cast<unsigned char>(std::sqrt(valueB) * brightenEdgeOffset));

          DOT("module:WhiteGoalPerceptor2:EdgesUnfiltered", bar.imageOrigin.x() + x, bar.imageOrigin.y() + y, ColorRGBA(value, value, value), ColorRGBA(value, value, value));
        }
      }
    }

    //Draw a black picture margin
    for(int y = 0; y < bar.imageHeight; y++)
    {
      DOT("module:WhiteGoalPerceptor2:EdgesUnfiltered", bar.imageOrigin.x() - 1, bar.imageOrigin.y() + y, ColorRGBA(0, 0, 0), ColorRGBA(0, 0, 0));
      DOT("module:WhiteGoalPerceptor2:EdgesUnfiltered", bar.imageOrigin.x() + bar.imageWidth, bar.imageOrigin.y() + y, ColorRGBA(0, 0, 0), ColorRGBA(0, 0, 0));
    }
    for(int x = 0; x < bar.imageWidth; x++)
    {
      DOT("module:WhiteGoalPerceptor2:EdgesUnfiltered", bar.imageOrigin.x() + x, bar.imageOrigin.y() - 1, ColorRGBA(0, 0, 0), ColorRGBA(0, 0, 0));
      DOT("module:WhiteGoalPerceptor2:EdgesUnfiltered", bar.imageOrigin.x() + x, bar.imageOrigin.y() + bar.imageHeight, ColorRGBA(0, 0, 0), ColorRGBA(0, 0, 0));
    }
  }

  validateGoalPost(bar);
}

void WhiteGoalPerceptor2::createGoalModel(const float& distance, PossiblePost& post)
{
  const float distanceInField = theFieldDimensions.yPosLeftGoal - theFieldDimensions.yPosRightGoal;

  const int midWidth = static_cast<int>(Geometry::getSizeByDistance(theCameraInfo, distanceInField, distance));
  const int midHeight = static_cast<int>(Geometry::getSizeByDistance(theCameraInfo, theFieldDimensions.goalHeight, distance));
  const int crossBarRadius = static_cast<int>(Geometry::getSizeByDistance(theCameraInfo, theFieldDimensions.crossBarRadius, distance));

  post.model.radius = static_cast<int>(Geometry::getSizeByDistance(theCameraInfo, theFieldDimensions.goalPostRadius, distance));

  post.model.outerWidth = midWidth + post.model.radius * 2;
  post.model.innerWidth = midWidth - post.model.radius * 2;
  post.model.innerHeight = midHeight - crossBarRadius;
  post.model.outerHeight = midHeight + crossBarRadius;

  post.model.angleInDeg = toDegrees(angleInRad);
  int errorThreshold = std::abs(90 - angleOffsetInDeg * 2);
  if(post.model.angleInDeg > errorThreshold || post.model.angleInDeg < -errorThreshold)
    post.model.angleInDeg -= 90;

  post.modelCreated = true;
}

bool WhiteGoalPerceptor2::sobel(PossiblePost& post)
{
  if(!post.modelCreated)
    return false;

  int *p, *pEnd;
  const Image::Pixel *pSrc;

  //ignore margin
  int start = post.imageOrigin.y();
  if(post.imageOrigin.y() == 0)
  {
    start = 1;
    ASSERT(post.edgeMap[0].empty()); // resize fills all elements with 0
    post.edgeMap[0].resize(post.imageWidth);
  }

  ASSERT(post.imageOrigin.y() + post.imageHeight < theImage.height);
  ASSERT(post.imageOrigin.x() + post.imageWidth < theImage.width);

  for(int y = start; y < post.imageOrigin.y() + post.imageHeight; y++)
  {
    std::vector<int>& pLine = post.edgeMap[y - post.imageOrigin.y()];
    ASSERT(pLine.empty()); // last element will be 0 due to resize
    pLine.resize(post.imageWidth);
    for(pSrc = theImage[y] + post.imageOrigin.x() + 1, p = &pLine[0], pEnd = p + post.imageWidth - 1; p < pEnd; p++, pSrc++)
      *p = sobelFast(pSrc, theImage.widthStep);
  }

  return true;
}

void WhiteGoalPerceptor2::nonMaximumSuppression(PossiblePost& post)
{
  int maximum;
  for(unsigned int y = 0; y < post.edgeMap.size(); y++)
  {
    //If the windowSize is greater than the actual image use half the imagesize as the windowsize to check the complete image
    //windowSizeR = static_cast<int>(post.houghImg[y].size() / 2) - 1;
    for(unsigned int x = 0; x < post.edgeMap[y].size(); x++)
    {
      maximum = -1;
      int start = std::max(static_cast<int>(0), static_cast<int>(x - post.model.radius));
      int end = std::min(static_cast<int>(post.edgeMap[y].size() - 1), static_cast<int>(x + post.model.radius));
      for(int j = start; j < end; j++)
      {
        if(maximum < post.edgeMap[y][j])
          maximum = post.edgeMap[y][j];
      }
      if (maximum > post.edgeMap[y][x])
        post.edgeMap[y][x] = 0;
    }
  }
}

void WhiteGoalPerceptor2::createLUT(const float& angleInRad)
{
  for(int i = -angleOffsetInDeg; i < angleOffsetInDeg; i++)
  {
    //sinLut[i] = sin(i + minAngle) same for cos
    float iInRad = Angle::fromDegrees(i) + angleInRad;
    sinLUTsA.push_back(std::sin(90_deg + iInRad));
    cosLUTsA.push_back(std::cos(90_deg + iInRad));
    sinLUT.push_back(std::sin(iInRad));
    cosLUT.push_back(std::cos(iInRad));
  }
}

inline float WhiteGoalPerceptor2::calculateAngle()
{
  float rollingShutter = 0;
  float bodyRotationAngleCompare;
  float returnAngle;

  //rollingshutter calculated by using fromCorrectedApprox to calculcate two lines and get the value
  Vector2f inImage1 = theImageCoordinateSystem.fromCorrectedApprox(Vector2i(50, 50));
  Vector2f inImage2 = theImageCoordinateSystem.fromCorrectedApprox(Vector2i(50, 100));
  rollingShutter = static_cast<float>(atan2(inImage2.y() - inImage1.y(), inImage2.x() - inImage1.x())) - pi / 2;

  //image in bodyrotation
  bodyRotationAngleCompare = theCameraMatrix.rotation.getXAngle();

  returnAngle = -rollingShutter - bodyRotationAngleCompare;

  return 0; //return 0 for now later: -rollingShutter + bodyRotationAngleCompare;
}

void WhiteGoalPerceptor2::otsuThreshold(PossiblePost& post)
{
  std::vector<int> histogram(256,0);
  for (int y = 0; y < post.imageHeight; y++)
  {
    for (int x = 0; x < post.imageWidth; x++)
    {
      int value = static_cast<int>(std::sqrt(post.edgeMap[y][x]));
      ASSERT(value > 0);
      ASSERT(histogram.size() > static_cast<unsigned int>(value));
      histogram[value]++;
    }
  }

  float meanThreshold = 0;
  int threshold = 0;
  int background = 0;
  int foreground = 0;
  int sumBackground = 0;
  float max = 0;

  for (unsigned int i = 1; i < histogram.size(); i++)
    meanThreshold += (i * histogram[i - 1]);

  for (unsigned int t = 0; t < histogram.size(); t++)
  {
    background += histogram[t];
    if (background == 0)
      continue;

    foreground = (post.imageWidth*post.imageHeight) - background;
    if (foreground == 0)
      break;

    sumBackground += t * histogram[t];

    float meanDiff = (sumBackground / background - (meanThreshold - sumBackground) / foreground);
    float tmp = meanDiff * meanDiff * static_cast<float>(background * foreground);

    if (tmp > max)
    {
      max = tmp;
      threshold = t;
    }
  }
  DRAWTEXT("module:WhiteGoalPerceptor2:Threshold", post.base.x(), post.base.y() + 5, 12, ColorRGBA::red, threshold);

  post.threshold = threshold;
}

inline bool WhiteGoalPerceptor2::isGreenInSurrounding(const int& x, const int& y)
{
  if(y >= theImage.height - 1 || y < 1 || x < 1 || x >= theImage.width - 1) return false;

  ASSERT(y >= 1 && y < theImage.height - 1 && x >= 1 && x < theImage.width - 1);
  return theColorTable[theImage[y][x]].is(ColorClasses::green) ||
    (y < theImage.height - 1 && theColorTable[theImage[y + 1][x]].is(ColorClasses::green)) ||
    (y > 0 && theColorTable[theImage[y - 1][x]].is(ColorClasses::green)) ||
    (x < theImage.width - 1 && theColorTable[theImage[y][x + 1]].is(ColorClasses::green)) ||
    (x > 0 && theColorTable[theImage[y][x - 1]].is(ColorClasses::green));
}

MAKE_MODULE(WhiteGoalPerceptor2, perception)
