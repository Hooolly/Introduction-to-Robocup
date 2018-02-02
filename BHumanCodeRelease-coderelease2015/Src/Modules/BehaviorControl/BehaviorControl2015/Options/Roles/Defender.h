/** A test defender option - circular */
option(Defender)
{
	// Debug
    Annotation("robotPose deviation: ", theRobotPose.deviation);
    Annotation("robotPose validity: ", theRobotPose.validity);
	
    common_transition{
        if (theRobotPose.deviation > libCodeRelease.maxDeviation || theRobotPose.validity < libCodeRelease.minValidity)
            goto localizeSelf;
    }
    
    initial_state(start)
    {
        transition
        {
            if(state_time > 3000) // TODO remove delay before starting
                goto walkToDefenseArea;
	    if(state_time > 6000)
		goto kickNew;
        }
        action
        {
            //theHeadControlMode = HeadControl::lookLeftAndRight;
			headControlTurnAngle = (float) 0.3;
            //WalkAtSpeedPercentage(Pose2f(0.5f, 1.f, 1.f)); headControlTurnAngle = float(0.5);// turn in circles to locate yourself
            //Stand();
        }
    }

    


    state(walkToDefenseArea)
    {
        transition
        { // go to searchForBall if you've reached the goal
            // currentPos - goalPos
            Vector2f ownPosFieldCoordinates = libCodeRelease.robotToField(Vector2f(0.f, 0.f));
            float distance = libCodeRelease.distanceBetween(ownPosFieldCoordinates,
                                                            libCodeRelease.centerDefenseAreaFieldCoordinates);
            if(distance < 50.f) // after reaching the center of the penalty area, face toward the opponent side
                goto faceOpponentDirection;
        }
        action
        {
            //theHeadControlMode = HeadControl::lookLeftAndRight;
            headControlTurnAngle = (float)2.0;
            Vector2f centerDefenseAreaRobotCoordinates = libCodeRelease.fieldToRobot(
                libCodeRelease.centerDefenseAreaFieldCoordinates); // convert to relative coordinates
            Pose2f target = Pose2f(centerDefenseAreaRobotCoordinates.angle(),
                                   centerDefenseAreaRobotCoordinates); // turn to face in the right direction
            WalkToTarget(Pose2f(50.f, 50.f, 50.f), target );
        }
    }
	//another region for defending
	state(walkToNewArea)
    {
        transition
        { // go to searchForBall if you've reached the goal
            // currentPos - goalPos
            Vector2f ownPosFieldCoordinates = libCodeRelease.robotToField(Vector2f(0.f, 0.f));
            float distance = libCodeRelease.distanceBetween(ownPosFieldCoordinates,
                                                            libCodeRelease.centerDefenseAreaFieldCoordinates + Vector2f(2000.f, 0.f));
            if(distance < 50.f) // after reaching the center of the penalty area, face toward the opponent side
                goto faceOpponentDirectionNew;
        }
        action
        {
            //theHeadControlMode = HeadControl::lookLeftAndRight;

			Vector2f target_2f = theBallModel.estimate.position;
			Vector3f target_3f(target_2f[0], target_2f[1], 0.f);
			SetHeadTargetOnGround(target_3f);


            headControlTurnAngle = (float)2.0;
            Vector2f centerDefenseAreaRobotCoordinates = libCodeRelease.fieldToRobot(
                libCodeRelease.centerDefenseAreaFieldCoordinates + Vector2f(2000.f, 0.f) ); // convert to relative coordinates
            Pose2f target = Pose2f(centerDefenseAreaRobotCoordinates.angle(),
                                   centerDefenseAreaRobotCoordinates); // turn to face in the right direction
            WalkToTarget(Pose2f(50.f, 50.f, 50.f), target );
        }
    }


    state(faceOpponentDirection)
    {
        transition
        {
            // if (action_done)
            //	goto defendGoal;
            if(std::abs(libCodeRelease.fieldToRobot(libCodeRelease.centerOpponentGroundLine).angle()) < 5_deg) {
                goto defend;
            }
	    
        }
        action
        { // face toward the center of the field
            Annotation("Turning towards center of field...");
            //theHeadControlMode = HeadControl::lookLeftAndRight;
            headControlTurnAngle = (float)0.5; // in radians
            Vector2f relativeCenterField =
                libCodeRelease.fieldToRobot(libCodeRelease.centerField); // transform to robot coordinates
            WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(relativeCenterField.angle(), 0.f, 0.f));
        }
    }

    state(faceOpponentDirectionNew)
    {
        transition
        {
            // if (action_done)
            //	goto defendGoal;
            if(std::abs(libCodeRelease.fieldToRobot(libCodeRelease.centerOpponentGroundLine).angle()) < 5_deg) {
                goto defendNew;
            }
	    
        }
        action
        { // face toward the center of the field
            Annotation("Turning towards center of field...");
            //theHeadControlMode = HeadControl::lookLeftAndRight;
            headControlTurnAngle = (float)0.5; // in radians
            Vector2f relativeCenterField =
                libCodeRelease.fieldToRobot(libCodeRelease.centerField); // transform to robot coordinates
            WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(relativeCenterField.angle(), 0.f, 0.f));
        }
    }

    state(defend)
    { // place yourself on a line between ball and goal center, stay on goal x position
        transition
        {
            // ball close to goalkeeper -> kick away
            float distance = libCodeRelease.distanceBetween(
                libCodeRelease.robotToField(theBallModel.estimate.position),
                libCodeRelease.robotToField(Vector2f(0.f, 0.f))); // ballPosition - robotPosition
		Vector2f posBallFieldCoordinates = libCodeRelease.robotToField(theBallModel.estimate.position);            
		if(distance < 6000.f && std::abs(posBallFieldCoordinates[1]) < 3000.f && posBallFieldCoordinates[0] < 200.f ) {
                goto turnToBall;
            }
		if(distance > 6000.f) {
                goto walkToNewArea;
            }
            if(libCodeRelease.timeSinceBallWasSeen() > 3000)
                goto searchForBall;
            // otherwise stay in this state, unless you lose track of the ball
        }
        action
        {
		Vector2f Ballpose = libCodeRelease.robotToField(theBallModel.estimate.position);
		Annotation("Ballpose: ", Ballpose[0]);		
		//face ball
		Vector2f target_2f = theBallModel.estimate.position;
		Vector3f target_3f(target_2f[0], target_2f[1], 0.f);
		SetHeadTargetOnGround(target_3f);

  	 		//theHeadControlMode = HeadControl::lookForward;
            //theHeadControlMode = HeadControl::lookLeftAndRight; // look left and right,but don't turn your head too much
            // headControlTurnAngle = 0.5;
            // first, get het field coordinates of the ball:
            Vector2f posBallFieldCoordinates = libCodeRelease.robotToField(theBallModel.estimate.position);
            // calculate distance to goal
            float distanceGoalToBall = libCodeRelease.distanceBetween(
                posBallFieldCoordinates, libCodeRelease.centerOwnGoalGroundLine); // in radians
            Annotation("distance ball to goal: ", distanceGoalToBall);
            
            float scalar = libCodeRelease.defenseRadius / distanceGoalToBall; // TODO if distanceGoalToBall is smaller than radius, weird things happen
            Annotation("scalar: ", scalar);
            
            // position yourself between ball and goal
            // first get coordinates of the ball relative to your own goal: ball,goal = ball,field + field,goal
            Vector2f posBallGoalCoordinates = posBallFieldCoordinates + (libCodeRelease.centerField - libCodeRelease.centerOwnGoalGroundLine);
            // then scale them to get the desired position of the defender (which is defined as on a circle relative to the own goal, that's why the previous step is needed)
            Vector2f defendPosition = Vector2f(posBallGoalCoordinates * scalar);
            // convert back to field coordinates: defender in field coordinates = defender,field = defender,goal + goal,field
            defendPosition = defendPosition + (libCodeRelease.centerOwnGoalGroundLine - libCodeRelease.centerField);
            
            defendPosition = libCodeRelease.fieldToRobot(defendPosition);
            Pose2f target = Pose2f(theBallModel.estimate.position.angle(), defendPosition); // target position in relative coordinates
            WalkToTarget(Pose2f(20.f, 20.f, 20.f), target);
        }
    }
	
	state(defendNew)
    { // place yourself on a line between ball and goal center, stay on goal x position
        transition
        {
            // ball close to goalkeeper -> kick away
            float distance = libCodeRelease.distanceBetween(
                libCodeRelease.robotToField(theBallModel.estimate.position),
                libCodeRelease.robotToField(Vector2f(0.f, 0.f))); // ballPosition - robotPosition
			Vector2f dis = libCodeRelease.robotToField(theBallModel.estimate.position) - libCodeRelease.robotToField(Vector2f(0.f, 0.f));
 Vector2f posBallFieldCoordinates = libCodeRelease.robotToField(theBallModel.estimate.position);
            if(distance < 3000.f && std::abs(posBallFieldCoordinates[1]) < 3000.f && posBallFieldCoordinates[0] < 200.f ) {
                goto turnToBallNew;
            }
			if( dis[0] < 0.f) {
                goto defend;
			}

            if(libCodeRelease.timeSinceBallWasSeen() > 2000)
                goto searchForBall;
            // otherwise stay in this state, unless you lose track of the ball
        }
        action
        {
	//face ball
		Vector2f target_2f = theBallModel.estimate.position;
		Vector3f target_3f(target_2f[0], target_2f[1], 0.f);
		SetHeadTargetOnGround(target_3f);  	 	

//theHeadControlMode = HeadControl::lookForward;
            //theHeadControlMode = HeadControl::lookLeftAndRight; // look left and right,but don't turn your head too much
            //headControlTurnAngle = 0.5;
            // first, get het field coordinates of the ball:
            Vector2f posBallFieldCoordinates = libCodeRelease.robotToField(theBallModel.estimate.position);
            // calculate distance to goal
            float distanceGoalToBall = libCodeRelease.distanceBetween(
                posBallFieldCoordinates, libCodeRelease.centerOwnGoalGroundLine); // in radians
            Annotation("distance ball to goal: ", distanceGoalToBall);
            
            float scalar = libCodeRelease.defenseRadius / distanceGoalToBall; // TODO if distanceGoalToBall is smaller than radius, weird things happen
            Annotation("scalar: ", scalar);
            
            // position yourself between ball and goal
            // first get coordinates of the ball relative to your own goal: ball,goal = ball,field + field,goal
            Vector2f posBallGoalCoordinates = posBallFieldCoordinates + (libCodeRelease.centerField - libCodeRelease.centerOwnGoalGroundLine);
            // then scale them to get the desired position of the defender (which is defined as on a circle relative to the own goal, that's why the previous step is needed)
            Vector2f defendPosition = Vector2f(posBallGoalCoordinates * scalar);
            // convert back to field coordinates: defender in field coordinates = defender,field = defender,goal + goal,field
            defendPosition = defendPosition + (libCodeRelease.centerOwnGoalGroundLine - libCodeRelease.centerField) + Vector2f(2000.f, 0.f);
            
            defendPosition = libCodeRelease.fieldToRobot(defendPosition);
            Pose2f target = Pose2f(theBallModel.estimate.position.angle(), defendPosition); // target position in relative coordinates
            WalkToTarget(Pose2f(20.f, 20.f, 20.f), target);
        }
    }
	
    
    state(walkBack)
    {
        transition
        { // go to searchForBall if you've reached the goal
            // currentPos - goalPos
            Vector2f ownPosFieldCoordinates = libCodeRelease.robotToField(Vector2f(0.f, 0.f));
            float distance = libCodeRelease.distanceBetween(ownPosFieldCoordinates,
                                                            libCodeRelease.centerDefenseAreaFieldCoordinates);
            if(distance < 50.f) // after reaching the center of the penalty area, face toward the opponent side
                goto faceOpponentDirection;
            if (theBallModel.estimate.position.norm()< 3000.f)
                goto turnToBall;
        }
        action
        {
            	 Vector2f posBallFieldCoordinates = libCodeRelease.robotToField(theBallModel.estimate.position);
		 Annotation("Ball position in Field: ", posBallFieldCoordinates[0]);
				//face ball
			//Vector2f target_2f = theBallModel.estimate.position;
			//Vector3f target_3f(target_2f[0], target_2f[1], 0.f);
			//SetHeadTargetOnGround(target_3f);
            headControlTurnAngle = (float)0.8;
            Vector2f centerDefenseAreaRobotCoordinates = libCodeRelease.fieldToRobot(
                libCodeRelease.centerDefenseAreaFieldCoordinates);          // convert to relative coordinates
            Pose2f target = Pose2f(0.0f, centerDefenseAreaRobotCoordinates);
            WalkToTarget(Pose2f(100.f, 100.f, 100.f), target);
        }
    }
	
	    state(walkBackNew)
    {
        transition
        { // go to searchForBall if you've reached the goal
            // currentPos - goalPos
            Vector2f ownPosFieldCoordinates = libCodeRelease.robotToField(Vector2f(0.f, 0.f));
            float distance = libCodeRelease.distanceBetween(ownPosFieldCoordinates,
                                                            libCodeRelease.centerDefenseAreaFieldCoordinates);
			Vector2f dis = libCodeRelease.robotToField(theBallModel.estimate.position) - libCodeRelease.robotToField(Vector2f(0.f, 0.f));
			if( dis[0] < 0.f) {
                goto walkBack;
			}
            if(distance < 50.f) // after reaching the center of the penalty area, face toward the opponent side
                goto faceOpponentDirectionNew;
            if (theBallModel.estimate.position.norm()< 1000.f)
                goto turnToBallNew;
        }
        action
        {
            //theHeadControlMode = HeadControl::lookLeftAndRight;
			//face ball
			//Vector2f target_2f = theBallModel.estimate.position;
			//Vector3f target_3f(target_2f[0], target_2f[1], 0.f);
			//SetHeadTargetOnGround(target_3f);

            headControlTurnAngle = (float)0.8;
            Vector2f centerDefenseAreaRobotCoordinates = libCodeRelease.fieldToRobot(
                libCodeRelease.centerDefenseAreaFieldCoordinates + Vector2f(2000.f, 0.f) );          // convert to relative coordinates
            Pose2f target = Pose2f(0.0f, centerDefenseAreaRobotCoordinates);
            WalkToTarget(Pose2f(100.f, 100.f, 100.f), target);
        }
    }

// all the following except searchForBall are from the Striker, to kick the ball properly
// only modification is direction: away from own goal instead of toward enemy goal
   

 state(turnToBall)
    {
        transition
        {   Vector2f posBallFieldCoordinates = libCodeRelease.robotToField(theBallModel.estimate.position);
	    Vector2f RobotPose = libCodeRelease.robotToField(Vector2f(0.f, 0.f));
            if(std::abs(theBallModel.estimate.position.angle()) < 5_deg && theBallModel.estimate.position.norm()<= 3000.f && std::abs(posBallFieldCoordinates[1]) <= 3000.f && posBallFieldCoordinates[0] < 200.f)
                goto walkToBall;
            
		//Vector2f Ballpose = libCodeRelease.robotToField(theBallModel.estimate.position);
	    
	    if(theBallModel.estimate.position.norm() > 3000.f || std::abs(posBallFieldCoordinates[1]) > 3000.f || posBallFieldCoordinates[0] >= 200.f) //if someone else kicks the ball away, go back to defending
		                
		goto defend;
            if(libCodeRelease.timeSinceBallWasSeen() > 3000)
                goto walkBack;
        }
        action
        {
	//face ball
			Vector2f target_2f = theBallModel.estimate.position;
			Vector3f target_3f(target_2f[0], target_2f[1], 0.f);
			SetHeadTargetOnGround(target_3f);
            //theHeadControlMode = HeadControl::lookLeftAndRight; 
			//headControlTurnAngle = float(0.5);
            WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));
        }
    }
	

	
	    state(turnToBallNew)
    {
        transition
        {
            Vector2f dis = libCodeRelease.robotToField(theBallModel.estimate.position) - libCodeRelease.robotToField(Vector2f(0.f, 0.f));
			if( dis[0] < 0.f) {
                goto walkBack;
			}

 Vector2f posBallFieldCoordinates = libCodeRelease.robotToField(theBallModel.estimate.position);
			if(std::abs(theBallModel.estimate.position.angle()) < 5_deg && theBallModel.estimate.position.norm()< 3000.f && std::abs(posBallFieldCoordinates[1]) < 3000.f && posBallFieldCoordinates[0] < 200.f)
                goto walkToBallNew;
            if(theBallModel.estimate.position.norm() >= 3000.f || std::abs(posBallFieldCoordinates[1]) >= 3000.f || posBallFieldCoordinates[0] > 200.f) //if someone else kicks the ball away, go back to defending
                goto defendNew;
            if(libCodeRelease.timeSinceBallWasSeen() > 4000)
                goto walkBackNew;
        }
        action
        {
	//face ball
			Vector2f target_2f = theBallModel.estimate.position;
			Vector3f target_3f(target_2f[0], target_2f[1], 0.f);
			SetHeadTargetOnGround(target_3f);
            //theHeadControlMode = HeadControl::lookLeftAndRight; 
			
//headControlTurnAngle = float(0.5);
            WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));
        }
    }

    state(walkToBall)
    {
        transition
        {	Vector2f posBallFieldCoordinates = libCodeRelease.robotToField(theBallModel.estimate.position);
		Vector2f RobotPose = libCodeRelease.robotToField(Vector2f(0.f, 0.f));
            if(theBallModel.estimate.position.norm() >= 3000.f || RobotPose[0] >= 200.f || RobotPose[1] > 3000.f || std::abs(posBallFieldCoordinates[1]) >= 3000.f || posBallFieldCoordinates[0] >= 200.f) //if someone else kicks the ball away, go back to defending
                goto defend;
			/*if(theBallModel.estimate.position.norm() < 500.f)
                goto alignToGoal;*/




 		Vector2f Ballpose = libCodeRelease.robotToField(theBallModel.estimate.position);
		Annotation("RobotPose: ", RobotPose[0]);
		if( Ballpose[0] > -1412.f && Ballpose[0] <= 200.f && std::abs(Ballpose[1]) <= 1388.f && theBallModel.estimate.position.norm() < 500.f)	 
 			 goto alignToGoal_c;
 		
		if( Ballpose[0] > -1412.f && Ballpose[0] <= 200.f && std::abs(Ballpose[1]) > 1388.f && theBallModel.estimate.position.norm() < 500.f)
               		goto alignToGoal_l;
	
		if( (Ballpose[0] > -2460.f && Ballpose[0] <= -1412.f && theBallModel.estimate.position.norm() < 500.f) || (Ballpose[0] <= -2460.f && Ballpose[0] >= -3700.f && std::abs(Ballpose[1]) > 1500.f && theBallModel.estimate.position.norm() < 500.f))
              		goto alignToGoal_r;

		if( Ballpose[0] <= -2460.f && Ballpose[0] > -3700.f && std::abs(Ballpose[1]) <= 1500.f && theBallModel.estimate.position.norm() < 400.f)
			goto alignToGoal;

                if(libCodeRelease.timeSinceBallWasSeen() > 4000)
                	goto walkBack;
        }
        action
        {
            
		//face ball
		Vector2f target_2f = theBallModel.estimate.position;
		Vector3f target_3f(target_2f[0], target_2f[1], 0.f);
		SetHeadTargetOnGround(target_3f);
			//theHeadControlMode = HeadControl::lookLeftAndRight;
			//headControlTurnAngle = float(0.5);
		WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f(theBallModel.estimate.position.angle(), 0.f, 0.f));                
		WalkToTarget(Pose2f(100.f, 100.f, 100.f), Vector2f (theBallModel.estimate.position.x(), theBallModel.estimate.position.y()));
        }
    }
	
	state(walkToBallNew)
    {
        transition
        {
			
 Vector2f posBallFieldCoordinates = libCodeRelease.robotToField(theBallModel.estimate.position);
Vector2f dis = libCodeRelease.robotToField(theBallModel.estimate.position) - libCodeRelease.robotToField(Vector2f(0.f, 0.f));
			if( dis[0] < 0.f) {
                goto walkBack;
			}

		Vector2f RobotPose = libCodeRelease.robotToField(Vector2f(0.f, 0.f));
            if(theBallModel.estimate.position.norm() >= 3000.f || RobotPose[0] > 1825.f || RobotPose[1] > 3000.f || std::abs(posBallFieldCoordinates[1]) >= 3000.f || posBallFieldCoordinates[0] >= 200.f) //if someone else kicks the ball away, go back to defending
                goto defendNew;
			
 		Vector2f Ballpose = libCodeRelease.robotToField(theBallModel.estimate.position);
		Annotation("RobotPose: ", RobotPose[0]);
		if( std::abs(Ballpose[1]) > 1388.f && Ballpose[0] >= 0.f  && theBallModel.estimate.position.norm() < 500.f)	 
 			 goto alignToGoal_c;
 		
		if( Ballpose[1] > 1388.f && Ballpose[0] >= 0.f && theBallModel.estimate.position.norm() < 500.f)
               		goto alignToGoal_l;
	
		if( Ballpose[1] < -1388.f && Ballpose[0] >= 0.f && theBallModel.estimate.position.norm() < 500.f)
              		goto alignToGoal_r;

		if( Ballpose[0] < 0.f && std::abs(Ballpose[1]) <= 1388.f && theBallModel.estimate.position.norm() < 500.f)
			goto alignToGoal;
            if(libCodeRelease.timeSinceBallWasSeen() > 4000)
                goto walkBackNew;
        }
        action
        {
//face ball
		Vector2f target_2f = theBallModel.estimate.position;
		Vector3f target_3f(target_2f[0], target_2f[1], 0.f);
		SetHeadTargetOnGround(target_3f);
            //theHeadControlMode = HeadControl::lookLeftAndRight;
			
//headControlTurnAngle = float(0.5);
            WalkToTarget(Pose2f(50.f, 50.f, 50.f), theBallModel.estimate.position);
        }
    }

    state(alignToGoal)
    {
        transition
        {
            if(theBallModel.estimate.position.norm() >= 1000.f) //if someone else kicks the ball away, go back to defending
                goto defend;
            if(libCodeRelease.timeSinceBallWasSeen() > 3000)
                goto walkBack;
            if(std::abs( libCodeRelease.angleToX ) < 8_deg && std::abs(theBallModel.estimate.position.y()) < 400.f)
                goto alignBehindBall;
        }
        action
        {
//face ball
		Vector2f target_2f = theBallModel.estimate.position;

		Vector3f target_3f(target_2f[0], target_2f[1], 0.f);
		SetHeadTargetOnGround(target_3f);
            	         //theHeadControlMode = HeadControl::lookLeftAndRight; 
			//headControlTurnAngle = float(0.5);
		//Vector2f posBallFieldCoordinates = libCodeRelease.robotToField(theBallModel.estimate.position);                 
		WalkToTarget(Pose2f(100.f, 100.f, 100.f),
                         Pose2f( libCodeRelease.angleToX, //opposite side of own goal
                                theBallModel.estimate.position.x() - 300.f,
                                theBallModel.estimate.position.y() + 50.f) , 0); //disable collision avoidance
        }
    }

 


    state(alignToGoal_l)
    {
        transition
        {
            if(theBallModel.estimate.position.norm() >= 1000.f) //if someone else kicks the ball away, go back to defending
                goto defend;
            if(libCodeRelease.timeSinceBallWasSeen() > 3000)
                goto walkBack;
            if(std::abs(libCodeRelease.angleToGoal) < 8_deg && std::abs(theBallModel.estimate.position.y()) < 100.f)
                goto alignBehindBall_l;
        }
        action
        {
//face ball
		Vector2f target_2f = theBallModel.estimate.position;
		Vector3f target_3f(target_2f[0], target_2f[1], 0.f);
		SetHeadTargetOnGround(target_3f);
            	         //theHeadControlMode = HeadControl::lookLeftAndRight; 
			//headControlTurnAngle = float(0.5);
                WalkToTarget(Pose2f(100.f, 100.f, 100.f),
                         Pose2f(libCodeRelease.angleToGoal, //opposite side of own goal
                                theBallModel.estimate.position.x() - 400.f,
                                theBallModel.estimate.position.y() + 30.f) , 0); //disable collision avoidance
        }
    }

    state(alignToGoal_r)
    {
        transition
        {
            if(theBallModel.estimate.position.norm() >= 1000.f) //if someone else kicks the ball away, go back to defending
                goto defend;
            if(libCodeRelease.timeSinceBallWasSeen() > 2000)
                goto walkBack;
            if(std::abs(libCodeRelease.angleToX) < 8_deg && std::abs(theBallModel.estimate.position.y()) < 100.f)
                goto alignBehindBall_r;
        }
        action
        {
//face ball
		Vector2f target_2f = theBallModel.estimate.position;
		Vector3f target_3f(target_2f[0], target_2f[1], 0.f);
		SetHeadTargetOnGround(target_3f);
            	         //theHeadControlMode = HeadControl::lookLeftAndRight; 
			//headControlTurnAngle = float(0.5);
                WalkToTarget(Pose2f(100.f, 100.f, 100.f),
                         Pose2f(1.f + libCodeRelease.angleToX, //opposite side of own goal
                                theBallModel.estimate.position.x() - 200.f,
                                theBallModel.estimate.position.y() + 30.f) , 0); //disable collision avoidance
        }
    }

    state(alignToGoal_c)
    {
        transition
        {
            if(theBallModel.estimate.position.norm() >= 1000.f) //if someone else kicks the ball away, go back to defending
                goto defend;
            if(libCodeRelease.timeSinceBallWasSeen() > 3000)
                goto walkBack;
            if(std::abs(libCodeRelease.angleToBoarder) < 8_deg && std::abs(theBallModel.estimate.position.y()) < 100.f)
                goto alignBehindBall_c;
        }
        action
        {
//face ball
		Vector2f target_2f = theBallModel.estimate.position;
		Vector3f target_3f(target_2f[0], target_2f[1], 0.f);
		SetHeadTargetOnGround(target_3f);
            	         //theHeadControlMode = HeadControl::lookLeftAndRight; 
			//headControlTurnAngle = float(0.5);
                WalkToTarget(Pose2f(100.f, 100.f, 100.f),
                         Pose2f(libCodeRelease.angleToBoarder, //opposite side of own goal
                                theBallModel.estimate.position.x() - 400.f,
                                theBallModel.estimate.position.y() + 30.f), 0); //disable collision avoidance
        }
    }

	
	    state(alignToGoalNew)
    {
        transition
        {
			Vector2f dis = libCodeRelease.robotToField(theBallModel.estimate.position) - libCodeRelease.robotToField(Vector2f(0.f, 0.f));
			if( dis[0] < 0.f) {
                goto walkBack;
			}
            if(theBallModel.estimate.position.norm() >= 1000.f) //if someone else kicks the ball away, go back to defending
                goto defendNew;
            if(libCodeRelease.timeSinceBallWasSeen() > 3000)
                goto walkBackNew;
            if(std::abs(libCodeRelease.angleToOwnGoal + 3.14f) < 8_deg && std::abs(theBallModel.estimate.position.y()) < 100.f)
                goto alignBehindBallNew;
        }
        action
        {
		//face ball
		Vector2f target_2f = theBallModel.estimate.position;
		Vector3f target_3f(target_2f[0], target_2f[1], 0.f);
		SetHeadTargetOnGround(target_3f);
            //theHeadControlMode = HeadControl::lookLeftAndRight; 
			//headControlTurnAngle = float(0.5);
            WalkToTarget(Pose2f(100.f, 100.f, 100.f),
                         Pose2f(3.14f + libCodeRelease.angleToOwnGoal, //opposite side of own goal
                                theBallModel.estimate.position.x() - 400.f,
                                theBallModel.estimate.position.y()), 0); //disable collision avoidance
        }
    }

 state(alignBehindBall)
    {
        transition
        {
            if(theBallModel.estimate.position.norm() >= 1000.f) //if someone else kicks the ball away, go back to defending
                goto defend;
            /*if(libCodeRelease.timeSinceBallWasSeen() > 2000)
                goto walkBack;*/
            if(libCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f) &&
               libCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f) &&
               std::abs(3.14f + libCodeRelease.angleToOwnGoal ) < 2_deg){
      	 		theHeadControlMode = HeadControl::lookForward;
                goto kick;}
        }
        action
        {
	//face ball
		Vector2f target_2f = theBallModel.estimate.position;
		Vector3f target_3f(target_2f[0], target_2f[1], 0.f);
		SetHeadTargetOnGround(target_3f);            
	//theHeadControlMode = HeadControl::lookLeftAndRight; headControlTurnAngle = float(0.5);
             WalkToTarget(Pose2f(80.f, 80.f, 80.f),
                         Pose2f( libCodeRelease.angleToX,
                                theBallModel.estimate.position.x() - 180.f,
                                theBallModel.estimate.position.y() + 80.f), 0);
		Vector2f RobotPose = libCodeRelease.robotToField(Vector2f(0.f, 0.f));
		Vector2f Ballpose = libCodeRelease.robotToField(theBallModel.estimate.position);
		if (RobotPose[0] < Ballpose[0] - 100.f)		
		WalkToTarget(Pose2f(80.f, 80.f, 80.f),
                         Pose2f(3.14f +libCodeRelease.angleToOwnGoal,
                                theBallModel.estimate.position.x() - 150.f,
                                theBallModel.estimate.position.y() - 30.f), 0);
		
        }
    }


 

state(alignBehindBall_l)
    {
        transition
        {
            if(theBallModel.estimate.position.norm() >= 1000.f) //if someone else kicks the ball away, go back to defending
                goto defend;
            /*if(libCodeRelease.timeSinceBallWasSeen() > 2000)
                goto walkBack;*/
            if(libCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f) &&
               libCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f) &&
               std::abs(libCodeRelease.angleToGoal) < 2_deg){
      	 		theHeadControlMode = HeadControl::lookForward;
                goto kick;}
        }
        action
        {
	//face ball
		Vector2f target_2f = theBallModel.estimate.position;
		Vector3f target_3f(target_2f[0], target_2f[1], 0.f);
		SetHeadTargetOnGround(target_3f);            
	//theHeadControlMode = HeadControl::lookLeftAndRight; headControlTurnAngle = float(0.5);
            WalkToTarget(Pose2f(80.f, 80.f, 80.f),
                         Pose2f(libCodeRelease.angleToGoal,
                                theBallModel.estimate.position.x() - 150.f,
                                theBallModel.estimate.position.y() - 30.f), 0);
        }
    }

state(alignBehindBall_r)
    {
        transition
        {
            if(theBallModel.estimate.position.norm() >= 1000.f) //if someone else kicks the ball away, go back to defending
                goto defend;
            /*if(libCodeRelease.timeSinceBallWasSeen() > 2000)
                goto walkBack;*/
            if(libCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f) &&
               libCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f) &&
               std::abs(libCodeRelease.angleToX) < 2_deg){
      	 		theHeadControlMode = HeadControl::lookForward;
                goto kick;}
        }
        action
        {
	//face ball
		Vector2f target_2f = theBallModel.estimate.position;
		Vector3f target_3f(target_2f[0], target_2f[1], 0.f);
		SetHeadTargetOnGround(target_3f);            
	//theHeadControlMode = HeadControl::lookLeftAndRight; headControlTurnAngle = float(0.5);
	 WalkToTarget(Pose2f(80.f, 80.f, 80.f),
                         Pose2f( libCodeRelease.angleToX + 1.f,
                                theBallModel.estimate.position.x() - 150.f,
                                theBallModel.estimate.position.y() + 30.f), 0);
		Vector2f RobotPose = libCodeRelease.robotToField(Vector2f(0.f, 0.f));
		Vector2f Ballpose = libCodeRelease.robotToField(theBallModel.estimate.position);
		if (RobotPose[0] < Ballpose[0])	
            WalkToTarget(Pose2f(80.f, 80.f, 80.f),
                         Pose2f(libCodeRelease.angleToX,
                                theBallModel.estimate.position.x() - 150.f,
                                theBallModel.estimate.position.y() - 30.f), 0);
        }
    }

state(alignBehindBall_c)
    {
        transition
        {
            if(theBallModel.estimate.position.norm() >= 1000.f) //if someone else kicks the ball away, go back to defending
                goto defend;
            /*if(libCodeRelease.timeSinceBallWasSeen() > 2000)
                goto walkBack;*/
            if(libCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f) &&
               libCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f) &&
               std::abs(libCodeRelease.angleToBoarder) < 2_deg){
      	 		theHeadControlMode = HeadControl::lookForward;
                goto kick;}
        }
        action
        {
	//face ball
		Vector2f target_2f = theBallModel.estimate.position;
		Vector3f target_3f(target_2f[0], target_2f[1], 0.f);
		SetHeadTargetOnGround(target_3f);            
	//theHeadControlMode = HeadControl::lookLeftAndRight; headControlTurnAngle = float(0.5);
            WalkToTarget(Pose2f(80.f, 80.f, 80.f),
                         Pose2f(libCodeRelease.angleToBoarder,
                                theBallModel.estimate.position.x() - 150.f,
                                theBallModel.estimate.position.y() - 30.f), 0);
        }
    }
	

	
    state(alignBehindBallNew)
    {
        transition
        {
			Vector2f dis = libCodeRelease.robotToField(theBallModel.estimate.position) - libCodeRelease.robotToField(Vector2f(0.f, 0.f));
			if( dis[0] < 0.f) {
                goto walkBack;
			}
            if(theBallModel.estimate.position.norm() >= 1000.f) //if someone else kicks the ball away, go back to defending
                goto defendNew;
            /*if(libCodeRelease.timeSinceBallWasSeen() > 2000)
                goto walkBack;*/
            if(libCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f) &&
               libCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f) &&
               std::abs(libCodeRelease.angleToOwnGoal + 3.14f) < 2_deg){
      	 		theHeadControlMode = HeadControl::lookForward;
                goto kickNew;}
        }
        action
        {
	//face ball
		Vector2f target_2f = theBallModel.estimate.position;
		Vector3f target_3f(target_2f[0], target_2f[1], 0.f);
		SetHeadTargetOnGround(target_3f);            
	//theHeadControlMode = HeadControl::lookLeftAndRight; headControlTurnAngle = float(0.5);
            WalkToTarget(Pose2f(80.f, 80.f, 80.f),
                         Pose2f(3.14f + libCodeRelease.angleToOwnGoal,
                                theBallModel.estimate.position.x() - 150.f,
                                theBallModel.estimate.position.y() + 30.f), 0);
        }
    }

    state(kick)
    {
        transition
        {
            if(state_time > 3000 || (state_time > 10 && action_done))
                goto walkBack;
        }
        action
        {	Vector2f Ballpose = libCodeRelease.robotToField(theBallModel.estimate.position);
            Annotation("Alive and Kickin'");
            //theHeadControlMode = HeadControl::lookForward;
//face ball
		Vector2f target_2f = theBallModel.estimate.position;
		Vector3f target_3f(target_2f[0], target_2f[1], 0.f);
		SetHeadTargetOnGround(target_3f);
            if(Ballpose[0] <= -2460.f && std::abs(Ballpose[1])<= 1500.f)	 
 			 
			InWalkKick(WalkRequest::left,
                       		    Pose2f(3.14f + libCodeRelease.angleToOwnGoal,
                                    theBallModel.estimate.position.x() - 160.f,
                                    theBallModel.estimate.position.y() - 55.f),
				    Pose2f( 1.7f, 1.7f, 1.7f));

		
		float dist = libCodeRelease.distanceBetween(
                libCodeRelease.robotToField(theBallModel.estimate.position),
                Vector2f(200.f, 0.f));

		if((Ballpose[0] >= -2460.f && Ballpose[0] < -1412.f) || (Ballpose[0] < -2460.f && Ballpose[0] >= -3700.f && std::abs(Ballpose[1])> 1500.f) ) //std::abs(Ballpose[1] < 1388.f) && dist < 1612.f  )
		
		
		InWalkKick(WalkRequest::left,
                       		    Pose2f(libCodeRelease.angleToX,
                                    theBallModel.estimate.position.x() - 160.f,
                                    theBallModel.estimate.position.y() - 55.f),
				    Pose2f( 2.f, 2.f, 2.f));
	        
		
      		//float speedkick = 10.f;
     		
		 
 		
	    if( std::abs(Ballpose[1]) > 1388.f && Ballpose[0] >= -1412.f && Ballpose[0] < 200.f)
                {InWalkKick(WalkRequest::left,
                       Pose2f(libCodeRelease.angleToGoal,
                              theBallModel.estimate.position.x() - 160.f,
                              theBallModel.estimate.position.y() - 55.f),
			      Pose2f(2.f , 2.f, 2.f));
        	}//if the ball is at the left side of the field	
	
  		float speedKick = libCodeRelease.kickspeed(libCodeRelease.distanceBetween(
                			libCodeRelease.robotToField(theBallModel.estimate.position),
                			Vector2f(200.f, 0.f)));	   
	 if(  std::abs(Ballpose[1]) <= 1388.f && Ballpose[0] >= -1312.f && Ballpose[0] < 200.f)
                InWalkKick(WalkRequest::left,
                       		    Pose2f(libCodeRelease.angleToBoarder,
                                    theBallModel.estimate.position.x() - 160.f,
                                    theBallModel.estimate.position.y() - 55.f),
				    Pose2f(speedKick,speedKick,speedKick));

	
        }
    }
    
	    state(kickNew)
    {
        transition
        {
            Vector2f dis = libCodeRelease.robotToField(theBallModel.estimate.position) - libCodeRelease.robotToField(Vector2f(0.f, 0.f));
			if( dis[0] < 0.f) {
                goto walkBack;
			}
			if(state_time > 3000 || (state_time > 10 && action_done))
                goto walkBackNew;
        }
        action
        {
            Annotation("Alive and Kickin'");
            
//face ball
		Vector2f target_2f = theBallModel.estimate.position;
		Vector3f target_3f(target_2f[0], target_2f[1], 0.f);
		SetHeadTargetOnGround(target_3f);
//theHeadControlMode = HeadControl::lookForward;
            InWalkKick(WalkRequest::left,
                       Pose2f(3.14f + libCodeRelease.angleToOwnGoal,
                              theBallModel.estimate.position.x() - 160.f,
                               theBallModel.estimate.position.y() - 55.f),
			      Pose2f(2.f, 2.f, 2.f));
        }
    }
    
    
 // here again non- kick states  
    state(searchForBall)
    {
        transition
        {
            if(libCodeRelease.timeSinceBallWasSeen() < 300)
				{theHeadControlMode = HeadControl::lookForward;
                goto defend;}
			if(state_time > 6000){
	            theHeadControlMode = HeadControl::lookForward;
				goto walkBack;} //go back to radius, then face opponent direction
        }
        action
        {
            WalkAtSpeedPercentage(Pose2f(0.3f, 0.f, 0.f));
            theHeadControlMode = HeadControl::lookLeftAndRight;headControlTurnAngle = float(0.5f);
        }
    }
    
    state(localizeSelf)
    {
        transition
        {

  			float distance = libCodeRelease.distanceBetween(
                libCodeRelease.robotToField(theBallModel.estimate.position),
                libCodeRelease.robotToField(Vector2f(0.f, 0.f))); // ballPosition - robotPosition
            if(theRobotPose.validity < libCodeRelease.maxDeviation && theRobotPose.validity > libCodeRelease.minValidity){
				if (distance > 4000.f)
				{
					theHeadControlMode = HeadControl::lookForward;
					goto walkBackNew;
				}
				else {
				theHeadControlMode = HeadControl::lookForward;
                goto walkBack;
				}
				} //searchForBall;}
        }
        action
        {
		Annotation("goal pose ", theFieldDimensions.xPosOpponentGroundline);	
            if (state_time < 5000){				//look left right
				Stand();
                theHeadControlMode = HeadControl::lookLeftAndRight; 
				headControlTurnAngle = float(0.5f);
				headControlTiltAngle = float(0.0f);
            }
		    else if (state_time< 12000){			//look up down
		        theHeadControlMode = HeadControl::lookLeftAndRight; 
				headControlTurnAngle = float(0.5f);
				headControlTiltAngle = float(0.0f);
		        WalkAtSpeedPercentage(Pose2f(0.3f, 0.05f, 0.05f));
		    }
			else if(state_time < 20000){
				Stand();
 				theHeadControlMode = HeadControl::lookLeftAndRight; 
				headControlTurnAngle = float(0.5f);
				headControlTiltAngle = float(0.0f);
            }
			else {
		            WalkAtSpeedPercentage(Pose2f(0.3f, 0.05f, 0.05f));
		            theHeadControlMode = HeadControl::lookLeftAndRight; 
					headControlTurnAngle = float(0.5f);
					headControlTiltAngle = float(0.0f);
	        }
        }
    }

    
}
