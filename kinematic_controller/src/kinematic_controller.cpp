#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"

float camX = 320;
float camY = 240;
float dataX = camX;
float dataY = camY;
int ball_detected = 0;
float change_in_length_1, change_in_length_2, change_in_length_3, change_in_length_4;

int motor_acceleration = 2000;
int stepperMaxSpeed = 300;

void perform_calculations(void);

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
//	ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void cameraDataCallback(const geometry_msgs::Point::ConstPtr& data){
//	dataX = data->data.z;
//	dataY = data->y;
//	perform_calculations();
}


void restore_default_speeds(void) {
}
void cameraDataCallback(const geometry_msgs::Point::ConstPtr& data){
	dataX = data->data.z;
	dataY = data->y;
	perform_calculations();
}


void endStopCheck(void) {
	((dataX < 160) && (current_x <= minX)) ? endstop1 = true : endstop1 = false;
	((dataX > 160) && (current_x >= maxX)) ? endstop2 = true : endstop2 = false;
	((dataY < 120) && (current_y <= minY)) ? endstop3 = true : endstop3 = false;
	((dataY > 120) && (current_y >= maxY)) ? endstop4 = true : endstop4 = false;
}


bool endStopCheck2(void) {
	recalculate_gripper_position();

	if ((dataX > 160) && (current_x >= (maxX - (xBorder / 2)))) {
		endStopX = true;
		camX = 0;
	} else if ((dataX < 160) && (current_x <= (minX + (xBorder / 2)))) {
		camX = 0;
		endStopX = true;
	} else {
		endStopX = false;
	}



	if ((dataY > 120) && (current_y >= (maxY - (yBorder / 2)))) {
		camY = 0;
		endStopY = true;
	}
	else if ((dataY < 120) && (current_y <= (minY + (yBorder / 2)))) {
		camY = 0;
		endStopY = true;
	} else {
		endStopY = false;
	}

	if (endStopY || endStopX) {
		endStop = true;
	} else {
		endStop = false;
	}
	return endStop;

}

float maxSpeed1;
void calculate_speeds(float d1, float d2, float d3, float d4) {

	float diff1 = abs(d1);
	float diff2 = abs(d2);
	float diff3 = abs(d3);
	float diff4 = abs(d4);
	maxSpeed1 = max(diff1, diff2);
	maxSpeed1 = max(maxSpeed1, diff3);
	maxSpeed1 = max(maxSpeed1, diff4);
	float speedMult1 = d1 / maxSpeed1;
	float speedMult2 = d2 / maxSpeed1;
	float speedMult3 = d3 / maxSpeed1;
	float speedMult4 = d4 / maxSpeed1;

	acceleration_scale();
	float speed1 = round(maxSpeedRamp * speedMult1);
	float speed2 = round(maxSpeedRamp * speedMult2);
	float speed3 = round(maxSpeedRamp * speedMult3);
	float speed4 = round(maxSpeedRamp * speedMult4);
}




void set_speeds (void);

float maxSpeedRamp = 0;
float beta = 0.9999;
void acceleration_scale(void) {
	if ((camX == 0) && (camY == 0)) {
		maxSpeedRamp = 0;
	} else {
		maxSpeedRamp = beta * maxSpeedRamp + (1 - beta) * stepperSpeed;
		quat_msg2.w = maxSpeedRamp;
	}
}


void perform_calculations(void) {

	camX = (dataX - 320);
	camY = (dataY - 240);

	recalculate_gripper_position();
	check_boundaries();



	ball_detected = 0;

  //scale (x,y) co-ordinates
	if ((ball_detected == 0)) {


    //find new lengths
		calculate_lengths(camX, camY);

		calculate_speeds(change_in_length_1, change_in_length_2, change_in_length_3, change_in_length_4);
		set_speeds();

    //find step counts
		float steper_counts_1 = length2Step(change_in_length_1);
		float steper_counts_2 = length2Step(change_in_length_2);
		float steper_counts_3 = length2Step(change_in_length_3);
		float steper_counts_4 = length2Step(change_in_length_4);


    //set motor positions
		steper_counts_1 += stepper1.currentPosition();
		steper_counts_2 += stepper2.currentPosition();
		steper_counts_3 += stepper3.currentPosition();
		steper_counts_4 += stepper4.currentPosition();


    //stepper.move stuff was here
    //stepper.move stuff was here
    //stepper.move stuff was here
    //stepper.move stuff was here


		endStopCheck2();
		calculation_flag = false;

	}
}

void endStopCal(void) {

	recalculate_gripper_position();

	if (((camX < 0) && (current_x <= (minX + 15))) || ((camX > 0) && (current_x >= (maxX - 15)))
		|| ((camY < 0) && (current_y <= (minY + 15)  )) || ((camY > 0) && (current_y >= (maxY - 15)))) {
		endStop = true;
    //This was not commented in the final working arduino code
    // stepper1.move(0);//This was not commented in the final working arduino code
    // stepper2.move(0);//This was not commented in the final working arduino code
    // stepper3.move(0);//This was not commented in the final working arduino code
    // stepper4.move(0);//This was not commented in the final working arduino code
	//This was not commented in the final working arduino code
}
else
	endStop = false;
}


int hyst = 2;
int tolerance = 5;
void check_boundaries(void) {
	camX = checkAxisBounds(camX, current_x, (minX + 10), (maxX - 10));
	camY = checkAxisBounds(camY, current_y, (minY + 10), (maxY - 10));
	quat_msg2.x = camX;
	quat_msg2.y = camY;
	camX = checkAxisDeadBand(camX, tolerance);
	camY = checkAxisDeadBand(camY, tolerance);
	quat_msg2.z = camX;
  //quat_msg2.w = camY;
}

float checkAxisBounds(float cam, float current, float minimum, float maximum) {
	if ((current <= minimum) && (cam < 0)) {
		boundFlag = true;
		return (minimum - current - hyst);
	} else if ((current >= maximum) && (cam > 0)) {
		boundFlag = true;
		return (maximum - current + hyst);
	} else {
		boundFlag = false;
		return cam;
	}
}

float checkAxisDeadBand(float location, int iTolerance) {
	if ((location < tolerance) && (location > (tolerance * (-1)))) {
		deadFlag = true;
		return 0;
	} else {
		deadFlag = false;
		return location;
	}
}


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
	ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
	ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
	ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
	ros::Subscriber cameraSub = n.subscribe("coordinate_send_topic", 1000, cameraDataCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.


   */

	while (ros::ok()){
	ros::spinOnce();


	}
	//	perform_calculations();
	//endStopCheck2();


return 0;
}
