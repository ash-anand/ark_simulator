#include "subscribe_pid_errors.h"

Subscribe_pid_errors::Subscribe_pid_errors(Shared_Memory* shared_memory,
					   threadGUI* t_gui) {
	this->shared_memory = shared_memory;
	this->t_gui = t_gui;

	kpx = 600.0;    // set values # 600
	kix = 0.0;      // set values # 1.0
	kdx = 180.0;    // set values # 180
	kpy = 600.0;    // set values # 600
	kiy = 0.0;      // set values # 1.0
	kdy = 180.0;    // set values # 180
	kppsi = 200.0;  // set values # 600
	kipsi = 0.001;  // set values # 1.0
	kdpsi = 80.0;   // set values # 180
	accelmax_x = 0.45;
	accelmax_y = 0.45;
	deaccelmax_x = 0.1;
	deaccelmax_y = 0.1;
	vmax_x = 0.5;
	vmax_y = 0.5;
	inr = 0.02;

	prev_time = 0.0;
	cout_prev_time = 0.0;
	x3 = (vmax_x * vmax_x) / (2 * deaccelmax_x) + inr;
	y3 = (vmax_y * vmax_y) / (2 * deaccelmax_y) + inr;
	sumerrorvx = 0.0;
	sumerrorvy = 0.0;
	sumerrorpsi = 0.0;
	preverrorvx = 0.0;
	preverrorvy = 0.0;
	preverrorpsi = 0.0;
}

long Subscribe_pid_errors::getMilliSecs() {
	timeval t;
	gettimeofday(&t, NULL);
	return t.tv_sec * 1000 + t.tv_usec / 1000;
}

void Subscribe_pid_errors::pidErrorsCb(const ark_msgs::PidErrorsConstPtr& msg) {
	long current_time = getMilliSecs();

	if (prev_time == 0) {
		prev_time = current_time;
		cout_prev_time = current_time;
		channel12y_mid = ((this->t_gui->gui->rc_maxlimits[0] -
				   this->t_gui->gui->rc_minlimits[0]) /
				  2) +
				 this->t_gui->gui->rc_minlimits[0];
		channel12x_mid = ((this->t_gui->gui->rc_maxlimits[1] -
				   this->t_gui->gui->rc_minlimits[1]) /
				  2) +
				 this->t_gui->gui->rc_minlimits[1];
		channel34y_mid = ((this->t_gui->gui->rc_maxlimits[2] -
				   this->t_gui->gui->rc_minlimits[2]) /
				  2) +
				 this->t_gui->gui->rc_minlimits[2];
		channel34x_mid = ((this->t_gui->gui->rc_maxlimits[3] -
				   this->t_gui->gui->rc_minlimits[3]) /
				  2) +
				 this->t_gui->gui->rc_minlimits[3];
		return;
	}
	double del_time = (current_time - prev_time) / 1000.0;
	if (this->shared_memory->getSharedControl() &&
	    this->shared_memory
		->getOverride())  // this->shared_memory->getSharedControl() &&
	{
		float errorx = msg->dx;
		float errory = msg->dy;
		float errorpsi = msg->dpsi;
		float errorz = msg->dz;
		float velx = msg->vx;
		float vely = msg->vy;
		float r_vx_target = 0;
		float r_vy_target = 0;
		// std::cout << errorx <<std::endl;

		int x_direction = (errorx > 0) ? -1 : 1;
		int y_direction = (errory > 0) ? -1 : 1;
		float mod_errorpsi = fabs(errorpsi);

		if (errorpsi > 0) {
			errorx = msg->dx * cos(mod_errorpsi) +
				 msg->dy * sin(mod_errorpsi);
			errory = -msg->dx * sin(mod_errorpsi) +
				 msg->dy * cos(mod_errorpsi);
		}

		else {
			errorx = msg->dx * cos(mod_errorpsi) -
				 msg->dy * sin(mod_errorpsi);
			errory = msg->dx * sin(mod_errorpsi) +
				 msg->dy * cos(mod_errorpsi);
		}

		errorx = fabs(errorx);
		errory = fabs(errory);

		if (errorx < inr) {
			targetv_x = 0;
			sumerrorvx = 0;
		} else {
			if (errorx <= x3) {
				targetv_x = sqrt(
				    fabs(vmax_x * vmax_x -
					 2 * deaccelmax_x * (x3 - errorx)));
				// std::cout<<x3<<" : "<<x_direction<<std::endl;
			} else
				targetv_x = vmax_x;
			targetv_x = targetv_x * x_direction;
			r_vx_target = targetv_x;
			// if (fabs(targetv_x - velx) > accelmax_x * del_time)
			// targetv_x = accelmax_x * x_direction * del_time +
			// velx;
		}
		if (errory < inr) {
			targetv_y = 0;
			sumerrorvy = 0;
		} else {
			if (errory <= y3) {
				targetv_y = sqrt(
				    fabs(vmax_y * vmax_y -
					 2 * deaccelmax_y * (y3 - errory)));
				// std::cout<<x3<<" : "<<x_direction<<std::endl;
			} else
				targetv_y = vmax_y;
			targetv_y = targetv_y * y_direction;
			r_vy_target = targetv_y;
			// if (fabs(targetv_y - vely) > accelmax_y * del_time)
			// targetv_y = accelmax_y * y_direction * del_time +
			// vely;
		}

		float errorvx = targetv_x - velx;
		float errorvy = targetv_y - vely;

		// Alt PID
		if (fabs(errorz) > 0.2) {
			// if (errorz > 0)
			// this->t_gui->gui->channel34->setYValue(channel34y_mid
			// - 100);
			// else if (errorz < 0)
			// this->t_gui->gui->channel34->setYValue(channel34y_mid
			// + 100);
		}
		// else this->t_gui->gui->channel34->setYValue(channel34y_mid);
		// End Alt Pid

		// Alt YAW
		int channelx_limit = 100;
		if (fabs(errorpsi) > 0.1) {
			float PIDpsi = channel34x_mid -
				       (kppsi * errorpsi + kipsi * sumerrorpsi +
					(kdpsi * (preverrorpsi - errorpsi)));
			if (PIDpsi > (channel34x_mid + channelx_limit))
				PIDpsi = channel34x_mid + channelx_limit;
			if (PIDpsi < (channel34x_mid - channelx_limit))
				PIDpsi = channel34x_mid - channelx_limit;
			// this->t_gui->gui->channel34->setXValue(PIDpsi);
			// std::cout<<PIDvx<<std::endl;
			sumerrorpsi = sumerrorpsi + errorpsi;
			preverrorpsi = errorpsi;
		}
		// else this->t_gui->gui->channel34->setXValue(channel34x_mid);
		// End Alt Pid

		// X PID
		int channely_limit = 100;
		if (fabs(errorvx) > 0) {
			float PIDvx = (kpx * errorvx + kix * sumerrorvx +
				       (kdx * (preverrorvx - errorvx))) +
				      channel12y_mid;
			// if (this->t_gui->gui->record_pid)
			// std::cout<<"X_x.csv,"<<errorvx<<","<<sumerrorvx<<","<<(preverrorvx
			// - errorvx)<<std::endl;
			// if (this->t_gui->gui->record_pid)
			// std::cout<<"Y_x.csv,"<<this->t_gui->gui->channel12->getYaxis()<<",0"<<std::endl;
			if (PIDvx > (channel12y_mid + channely_limit))
				PIDvx = channel12y_mid + channely_limit;
			if (PIDvx < (channel12y_mid - channely_limit))
				PIDvx = channel12y_mid - channely_limit;
			this->t_gui->gui->channel12->setYValue(PIDvx);
			sumerrorvx = sumerrorvx + errorvx;
			preverrorvx = errorvx;
		} else
			this->t_gui->gui->channel12->setYValue(channel12y_mid);
		// end X PID

		// Y PID
		if (fabs(errorvy) > 0) {
			float PIDvy = (kpy * errorvy + kiy * sumerrorvy +
				       (kdy * (preverrorvy - errorvy))) +
				      channel12x_mid;
			// if (this->t_gui->gui->record_pid)
			// std::cout<<"X_y.csv,"<<errorvy<<","<<sumerrorvy<<","<<(preverrorvy
			// - errorvy)<<std::endl;
			// if (this->t_gui->gui->record_pid)
			// std::cout<<"Y_y.csv,"<<this->t_gui->gui->channel12->getXaxis()<<",0"<<std::endl;
			if (PIDvy > (channel12x_mid + 200))
				PIDvy = channel12x_mid + 200;
			if (PIDvy < (channel12x_mid - 200))
				PIDvy = channel12x_mid - 200;
			this->t_gui->gui->channel12->setXValue(PIDvy);
			sumerrorvy = sumerrorvy + errorvy;
			preverrorvy = errorvy;
		} else
			this->t_gui->gui->channel12->setXValue(channel12x_mid);
		// end Y PID

		if (current_time - cout_prev_time > 0.5 &&
		    !this->t_gui->gui->record_pid) {
			std::cout << targetv_x << "," << velx << ","
				  << r_vx_target << "," << errorvx << "," << kpx
				  << std::endl;
			cout_prev_time = current_time;
		}
	}
	// Clean Up
	prev_time = current_time;
}
