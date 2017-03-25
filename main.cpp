/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Bullet header
#include "SoftDemo.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"
#include "DemoApplication.h"

/******************************************************************************\
* Copyright (C) 2012-2013 Leap Motion, Inc. All rights reserved.               *
* Leap Motion proprietary and confidential. Not for distribution.              *
* Use subject to the terms of the Leap Motion SDK Agreement available at       *
* https://developer.leapmotion.com/sdk_agreement, or another agreement         *
* between Leap Motion and you, your company or other organization.             *
\******************************************************************************/

//Leap header
#include <iostream>
#include "Leap.h"
using namespace Leap;

// create a global softbody object
SoftDemo* softDemo = new SoftDemo();
Leap::Frame	m_lastFrame;
bool bShouldRotate	= true;


// leap controller relative function
class SampleListener : public Listener {
  public:
    virtual void onInit(const Controller&);
    virtual void onConnect(const Controller&);
    virtual void onDisconnect(const Controller&);
    virtual void onExit(const Controller&);
    virtual void onFrame(const Controller&);
    virtual void onFocusGained(const Controller&);
    virtual void onFocusLost(const Controller&);
};

void SampleListener::onInit(const Controller& controller) {
  std::cout << "Initialized" << std::endl;
  softDemo->detection_switch = false;
  softDemo->spin_switch= 0; //spin switch
  softDemo->position_switch = 0; //spin switch
  softDemo->spin_rate = 5; // spin rate
  softDemo->force = 600; // spin rate
  softDemo->softbody_scale = 0.4f; //scale of pottery
}

void SampleListener::onConnect(const Controller& controller) {
  std::cout << "Connected" << std::endl;
  controller.enableGesture(Gesture::TYPE_CIRCLE);
  controller.enableGesture(Gesture::TYPE_KEY_TAP);
  controller.enableGesture(Gesture::TYPE_SCREEN_TAP);
  controller.enableGesture(Gesture::TYPE_SWIPE);
}

void SampleListener::onDisconnect(const Controller& controller) {
  //Note: not dispatched when running in a debugger.
  std::cout << "Disconnected" << std::endl;
}

void SampleListener::onExit(const Controller& controller) {
  std::cout << "Exited" << std::endl;
}

void SampleListener::onFrame(const Controller& controller) {
  // Get the most recent frame and report some basic information
  const Frame frame = controller.frame(); 
  /*//roate screen using gesture
  bShouldRotate = frame.rotationProbability(m_lastFrame) > 0.40;
  //std::cout<<frame.translationProbability(m_lastFrame)<<std::endl;
 // std::cout<<frame.rotationProbability(m_lastFrame)<<std::endl;
  //std::cout<<frame.rotationAngle(m_lastFrame)<<std::endl;
  //std::cout<<frame.rotationAxis(m_lastFrame)<<std::endl;
  if(bShouldRotate)
	  softDemo->leapCameraControl(frame.rotationAngle(m_lastFrame));
  m_lastFrame = frame;*/
  
  if(softDemo->detection_switch)    //if the detection switch is on
  if (!frame.hands().empty()) {
	for(int h=0;h<frame.hands().count();h++){
	    const Hand hand = frame.hands()[h];
	    // Check if the hand has any fingers
	    const FingerList fingers = hand.fingers();
		//implement with softbody by finger
	    if (!fingers.empty()) {
		for (int i = 0; i < fingers.count(); ++i) {
			//std::cout<<"hand "<<h<<" finger "<<fingers[i].id()<<" position "<<fingers[i].tipPosition()<<std::endl;
			//std::cout<<"hand "<<h<<" finger "<<fingers[i].id()<<" direciton "<<fingers[i].direction()<<std::endl;
			Vector dir = fingers[i].direction(); //	finger direction
			Vector pos = fingers[i].tipPosition(); // tip position
			float sp = fingers[i].tipVelocity().magnitude();; // finger speed
			float width = fingers[i].width();
			//std::cout<<"fingers "<<i<<"  "<<sp.magnitude()<<std::endl;
			//std::cout<<"fingers "<<i<<"  "<<width<<std::endl;
			softDemo->leapControl(softDemo,dir,pos,sp,width,true);
			}
		}
		//implement with softbody by palm
		float sp = hand.palmNormal().magnitude();
		softDemo->leapControl(softDemo,hand.palmNormal(),hand.palmPosition(),sp,hand.sphereRadius(),false);


		/*// Get the hand's sphere radius and palm position
		std::cout << "Hand sphere radius: " << hand.sphereRadius()
				<< " mm, palm position: " << hand.palmPosition() << std::endl;
		// Get the h	and's normal vector and direction
		const Vector normal = hand.palmNormal();
		const Vector direction = hand.direction();
		// Calculate the hand's pitch, roll, and yaw angles
		std::cout << "Hand pitch: " << direction.pitch() * RAD_TO_DEG << " degrees, "
				<< "roll: " << normal.roll() * RAD_TO_DEG << " degrees, "
				<< "yaw: " << direction.yaw() * RAD_TO_DEG << " degrees" << std::endl;*/
		}
	}
}

void SampleListener::onFocusGained(const Controller& controller) {
  std::cout << "Focus Gained" << std::endl;
  m_lastFrame = controller.frame();
}

void SampleListener::onFocusLost(const Controller& controller) {
  std::cout << "Focus Lost" << std::endl;
}


GLDebugDrawer	gDebugDrawer;

int main(int argc,char** argv)
{

    softDemo->initPhysics();
	softDemo->getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);

	// ----------Leap relative----------------
	// Create a sample listener and controller
	SampleListener listener;
	Controller controller;
	// Have the sample listener receive events from the controller
	controller.addListener(listener);


	glutmain(argc, argv,1024,78,"Bullet Physics Demo. http://bulletphysics.com",softDemo);
	//glutmain_only_with_bullet_leap(argc, argv,1024,768,"Bullet Physics Demo. http://bulletphysics.com",softDemo);


	std::cout << "Press Enter to quit..." << std::endl;
	std::cin.get(); 
   // Remove the sample listener when done
	controller.removeListener(listener);
	delete softDemo;

	return 0;

}