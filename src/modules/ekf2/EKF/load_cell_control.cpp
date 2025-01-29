#include "ekf.h"
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

//#include "python/ekf_derivation/generated/compute_load_cell_z_innov_var_and_k.h"
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/uORB.h>
# include <uORB/topics/external_wrench_estimation.h>

#include <stdexcept>  // Aggiunto per std::runtime_error




void Ekf::controlLoadCellFusion()
{

    if(_load_cell_buffer && !_control_status.flags.fake_pos && _control_status.flags.in_air){
		loadCellSample loadCell_sample;

		const float accel_z_raw = (_state.vel(2) - prev_state_vel_z) / _dt_ekf_avg;
		 
        updateAccelZBuffer(accel_z_raw);

        const float accel_z = filterAccelZ();

        const float vel_z_old = prev_state_vel_z;
        prev_state_vel_z = _state.vel(2);

		

		if (_load_cell_buffer->pop_first_older_than(_time_delayed_us, &loadCell_sample)) {

		 	fuseLoadCell(loadCell_sample,accel_z,vel_z_old);
		}
	}
}




void Ekf::fuseLoadCell(const loadCellSample &loadCell_Sample,const float accel_z,const float vel_z_old)
{
		
		//PARAMETRI UTILI

		 //const float R_FORCE = fmaxf(_params.load_cell_noise, 0.01f);
		const float R_FORCE = 0.1f; 
         //const float mass = _params.mass;
		//const float mass = 2.081f;
		const float mass_drone = 2.0643f;
		const float mass_arm = 0.017f;
		const float mass = mass_drone + mass_arm; 
		const float FORCE_THRESHOLD = 1.0f;
		const float bias_load_cell = - 0.0196;

		/*const float motor_constant = 8.54858e-6; // N·s^2
    	const float max_rot_velocity = 1000.0;    // rad/s
		const float num_motors = 4;
		float thrust_real_z = 0;*/
		
		
		
		
	/*CALCOLO DEL THRUST CON I SETPOINT
	
	vehicle_thrust_setpoint_s thrust_setpoint_data;

	if (vehicle_thrust_setpoint_sub.update(&thrust_setpoint_data)) {
    // Valori normalizzati del thrust lungo gli assi X, Y, Z
   
    float thrust_z_normalized = thrust_setpoint_data.xyz[2];

    //PX4_INFO("Thrust setpoint: X=%.4f, Y=%.4f, Z=%.4f", (double)thrust_x_normalized, (double)thrust_y_normalized, (double)thrust_z_normalized);


	const float MAX_THRUST = motor_constant*(max_rot_velocity*max_rot_velocity)*num_motors;

    // Se necessario, calcola il thrust reale
    	thrust_real_z =  (-thrust_z_normalized) * MAX_THRUST;
}*/





		//CALCOLO DEL THRUST
	    float total_thrust = compute_thrust_z();
		/*Vector3f thrust_body(0.0f, 0.0f, total_thrust);
		Vector3f thrust_NED = _state.quat_nominal.rotateVector(thrust_body);
		total_thrust = thrust_NED(2);*/


		//PREDIZIONE ACCELERAZIONE Z
		float estimated_force_z = predict_force_z(mass,total_thrust,accel_z);
		const float pred_acc = estimated_force_z/mass;


		//MISURA ACCELERAZIONE Z
	    float mea_force_z = -(loadCell_Sample.force(1) - bias_load_cell);
		mea_force_z_filtered = alpha_load_cell_filter * mea_force_z + (1.0f - alpha_load_cell_filter) * mea_force_z_filtered;
		const float mea_acc = mea_force_z/mass;


		//CALCOLO INNOVAZIONE
        _load_innov = pred_acc - mea_acc;
		//PX4_INFO("INNOVAZIONE: %.4f", static_cast<double>(_load_innov));
		

		//CALCOLO VARIANZA INNOVAZIONE E GUADAGNO DI KALMAN
		 const float H_vz = 0.1f;
		 const float H_pz = 0.0f;
		Vector24f H;
		H.setZero();
		H(6) = H_vz;  // Aggiorna solo v_z
		H(9) = H_pz;
		
		_load_innov_var = (H.transpose() * P * H)(0, 0) + R_FORCE;


		Vector24f Kfusion = P * H / _load_innov_var;


	 //UTILIZZO DI SYMFORCE
	
	/*
	const Vector24f state_vector_prev = getStateAtFusionHorizonAsVector();
	Vector24f Kfusion;
	matrix::Matrix<float, 1, 24> H;

	sym::ComputeLoadCellZInnovVarAndK(state_vector_prev, P, vel_z_old, R_FORCE, _dt_ekf_avg,total_thrust, mass, FLT_EPSILON, &H, &_load_innov_var, &Kfusion);
	*/


		//ATTIVA LA FUSIONE

	   //measurementUpdate(Kfusion, _load_innov_var, _load_innov);

	   if (fabs(loadCell_Sample.force(1)) > FORCE_THRESHOLD) {
        measurementUpdate(Kfusion, _load_innov_var, _load_innov);
		}



	const float gravity_force = mass*CONSTANTS_ONE_G;


	//PUBLISHER DI DEBUG
	struct external_wrench_estimation_s wrench_estimation = {};

	wrench_estimation.timestamp = loadCell_Sample.time_us; // Tempo corrente
	wrench_estimation.force_x = 0.0f;                  // Forza su X (fissata a 0)
	wrench_estimation.force_y = mea_acc;                  // Forza su Y (fissata a 0)
	wrench_estimation.force_z = estimated_force_z;     // Forza stimata su Z
	wrench_estimation.torque_x = total_thrust;                 // Momento torcente su X
	wrench_estimation.torque_y = accel_z*mass;                 // Momento torcente su Y
	wrench_estimation.torque_z = gravity_force;                 // Momento torcente su Z

if (_wrench_pub == nullptr) {
    
    _wrench_pub = orb_advertise(ORB_ID(external_wrench_estimation), &wrench_estimation);
} else {

    orb_publish(ORB_ID(external_wrench_estimation), _wrench_pub, &wrench_estimation);
}


}	




float Ekf::compute_thrust_z(){

	const float motor_constant = 8.54858e-6; // N·s^2
    //const float max_rot_velocity = 1000.0;    // rad/s
	const float num_motors = 4;

	float total_thrust = 0;


	actuator_outputs_s actuator_outputs_data;

    // Verifica se ci sono nuovi dati nel topic actuator_outputs
    if (actuator_outputs_sub.update(&actuator_outputs_data)) {
        //PX4_INFO("Calcolo del thrust totale dai valori di actuator_outputs:");

        // Cicla attraverso i motori
        for (uint32_t i = 0; i < actuator_outputs_data.noutputs && i < num_motors; ++i) { // Assumi che i primi 4 siano i motori
            float motor_speed = actuator_outputs_data.output[i]; // Velocità angolare del motore (rad/s)

            // Calcolo del thrust
            float motor_thrust = motor_constant * (motor_speed * motor_speed);
            total_thrust += motor_thrust;

            //PX4_INFO("Motore %d: velocità=%.2f rad/s, thrust=%.2f N", i, (double)motor_speed, (double)motor_thrust);
        }

    } else {
        PX4_WARN("Nessun dato disponibile da actuator_outputs");
    }


	return total_thrust;

}



float Ekf::predict_force_z(const float mass, float total_thrust, const float accel_z){


	const float gravity_force = mass*CONSTANTS_ONE_G;
		

		//UTILIZZO IMU
		//const imuSample imu_sample_delayed = _imu_buffer.get_oldest();
		//float imu_accel_x = imu_sample_delayed.delta_vel(0)/imu_sample_delayed.delta_vel_dt;
		//float imu_accel_y = imu_sample_delayed.delta_vel(1)/imu_sample_delayed.delta_vel_dt;
		//float imu_accel_z = imu_sample_delayed.delta_vel(2)/imu_sample_delayed.delta_vel_dt;
		//Vector3f accel_imu(imu_accel_x, imu_accel_y, imu_accel_z);
		//Vector3f accel_imu_NED = _state.quat_nominal.rotateVector(accel_imu);
		//imu_accel_z = accel_imu_NED(2) + CONSTANTS_ONE_G;
		//float estimated_force_z = mass*accel_z - total_thrust + gravity_force; //il thrust segue già la convenzione
		//float estimated_force_z = mass*(imu_accel_z) - total_thrust + gravity_force;


		//UTILIZZO ACCELERAZIONE CALCOLATA DA VELOCITÀ

		float estimated_force_z = mass*accel_z - total_thrust + gravity_force;


		return estimated_force_z;

}


void Ekf::updateAccelZBuffer(float accel_z) {
    // Aggiungi il nuovo valore al buffer
    accel_z_buffer.push_back(accel_z);

    // Mantieni la dimensione del buffer entro la finestra
   if (accel_z_buffer.size() > static_cast<std::size_t>(window_size)) {
        accel_z_buffer.pop_front();
    }
}



float Ekf::filterAccelZ() {
    if (accel_z_buffer.empty()) {
        throw std::runtime_error("Il buffer di accel_z è vuoto!");
    }

    float sum = 0.0f;

    // Somma i valori nel buffer
    for (const float value : accel_z_buffer) {
        sum += value;
    }

    // Restituisci la media
    return sum / accel_z_buffer.size();
}










