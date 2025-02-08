#include "ekf.h"
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

//#include "python/ekf_derivation/generated/compute_load_cell_z_innov_var_and_k.h"
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/uORB.h>
#include <uORB/topics/external_wrench_estimation.h>


#include <stdexcept>  // Aggiunto per std::runtime_error




void Ekf::controlLoadCellFusion()
{

    if(_load_cell_buffer && !_control_status.flags.fake_pos && _control_status.flags.in_air){
		loadCellSample loadCell_sample;

		/*const float accel_z_raw = (_state.vel(2) - prev_state_vel_z) / _dt_ekf_avg;
        updateAccelZBuffer(accel_z_raw);
        const float accel_z = filterAccelZ();
        const float vel_z_old = prev_state_vel_z;
        prev_state_vel_z = _state.vel(2);*/


		quaternionToRotationMatrix();

		compute_thrust_z();
        
        float total_thrust = getDelayedThrust();
		float f_z = predict_force_z(total_thrust);
		PX4_INFO("f_z = %f", (double)f_z);
		predictAugState(total_thrust,f_z);
		predictAugCovariance();
		


     

		//if (_load_cell_buffer->pop_first_older_than(_time_delayed_us, &loadCell_sample)) {

		updateLoadCell(loadCell_sample);
		//}


        prev_thrust = total_thrust;
		prev_augstate_accel = augstate.aug_accel;
        prev_augstate_pos = augstate.aug_pos;
        prev_augstate_vel = augstate.aug_vel;
		

		
	}
}





void Ekf::quaternionToRotationMatrix(){

	float q0 = _state.quat_nominal(0);
	float q1 = _state.quat_nominal(1);
	float q2 = _state.quat_nominal(2);
	float q3 = _state.quat_nominal(3);



    Rk(0,0) = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    Rk(0,1) = 2 * (q1 * q2 - q0 * q3);
    Rk(0,2) = 2 * (q1 * q3 + q0 * q2);

    Rk(1,0) = 2 * (q1 * q2 + q0 * q3);
    Rk(1,1)= q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3;
    Rk(1,2) = 2 * (q2 * q3 - q0 * q1);

    Rk(2,0) = 2 * (q1 * q3 - q0 * q2);
    Rk(2,1) = 2 * (q2 * q3 + q0 * q1);
    Rk(2,2) = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;


}





void Ekf::compute_thrust_z(){

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

		//total_thrust = - total_thrust;

		updateThrustBuffer(total_thrust);

    } else {
        PX4_WARN("Nessun dato disponibile da actuator_outputs");
    }

}






float Ekf::predict_force_z(float total_thrust){


	const float mass_drone = 2.0643f;
	const float mass_arm = 0.017f;
	const float mass = mass_drone + mass_arm; 
	

	Vector3f e3(0,0,1);
	Vector3f ak = augstate.aug_accel;

    //total_thrust = getDelayedThrust();

	matrix::Matrix<float, 3, 1> temp = (mass*(ak)) - (mass*CONSTANTS_ONE_G*e3 - total_thrust*Rk*e3);

	float fz = (e3.transpose()*Rk.transpose() * temp)(0,0);

	return fz;

}



void Ekf::predictAugState(float total_thrust, float f_z){


	const float mass_drone = 2.0643f;
	const float mass_arm = 0.017f;
	const float mass = mass_drone + mass_arm; 

	float Tprev = prev_thrust/mass;
    //total_thrust = getDelayedThrust();
	float T = total_thrust/mass;

	Vector3f ang_vel = retrieveAngularVelocity();
    Vector3f e3(0,0,1);

    
    augstate.aug_vel = prev_augstate_vel + _dt_ekf_avg*(prev_augstate_accel + CONSTANTS_ONE_G*e3);
	augstate.aug_pos = prev_augstate_pos + _dt_ekf_avg*prev_augstate_vel;
    
    

    //PX4_INFO("pos_z = %f", (double)augstate.aug_pos(2));

    //PX4_INFO("vel_z = %f", (double) augstate.aug_vel(2));
	
	augstate.aug_quat_nominal = _state.quat_nominal;
	augstate.aug_ang_vel = ang_vel;

	matrix::SquareMatrix<float, 3> Sw;
	


    Sw(0,0) = 0.0f;
    Sw(0,1) = - ang_vel(2);
    Sw(0,2) =  ang_vel(1);

    Sw(1,0) = ang_vel(2);
    Sw(1,1) = 0.0f;
    Sw(1,2) = -ang_vel(0);

    Sw(2,0) = - ang_vel(1);
    Sw(2,1) = ang_vel(0);
    Sw(2,2) = 0.0f;

	Vector3f temp1 = ((- T + Tprev)/_dt_ekf_avg) * Rk*e3;
	Vector3f temp2 =  - T*Rk*Sw*e3;
	Vector3f temp3 = f_z/mass * Rk * Sw * e3; 


	augstate.aug_accel = (prev_augstate_accel + _dt_ekf_avg*(temp1+temp2+temp3));
   

    
    


    //PX4_INFO("temp1 = %f", (double)temp1(2));
    //PX4_INFO("temp2 = %f", (double)temp2(2));
    //PX4_INFO("temp3  = %f", (double)temp3(2));

	//PX4_INFO("accel z = %f", (double)augstate.aug_accel(2));


}



void Ekf::predictAugCovariance(){



matrix::SquareMatrix<float, 9> Fk{};


matrix::Matrix3f I;
I.setIdentity();  


    Fk.slice<3,3>(0,0) = I;   
    Fk.slice<3,3>(0,3) = _dt_ekf_avg * I; 
    Fk.slice<3,3>(3,3) = I;    
    Fk.slice<3,3>(3,6) = _dt_ekf_avg * I; 
    Fk.slice<3,3>(6,6) = I;   


P_aug = Fk * P_aug * Fk.transpose();


}


void Ekf::updateLoadCell(const loadCellSample &loadCell_Sample){



	//PARAMETRI UTILI

		 //const float R_FORCE = fmaxf(_params.load_cell_noise, 0.01f);
		const float R_FORCE = 0.1f; 
         //const float mass = _params.mass;
		//const float mass = 2.081f;
		const float mass_drone = 2.0643f;
		const float mass_arm = 0.017f;
		const float mass = mass_drone + mass_arm; 
		//const float FORCE_THRESHOLD = 1.0f;
		const float bias_load_cell = - 0.0196;

		
		//MISURA ACCELERAZIONE Z
	    float mea_force_z_raw = -(loadCell_Sample.force(1) - bias_load_cell);
		updateMeasBuffer(mea_force_z_raw);
		float mea_force_z_filtered = filterForceMeas();
		float mea_acc = mea_force_z_filtered/mass;



		matrix::Vector<float, 9> H;
		Vector3f e3(0,0,1);
		Vector3f accel_component = (e3.transpose()*Rk.transpose()*mass).transpose();
		H(6) = accel_component(0);  // Aggiorna solo v_z
		H(7) = accel_component(1);
		H(8) = accel_component(2);
		
		_load_innov_var = (H.transpose() * P_aug * H)(0, 0) + R_FORCE;


		matrix::Vector<float, 9> Kfusion; 
		Kfusion = P_aug * H / _load_innov_var;

		

		Vector3f H_pos = H.slice<3,1>(0,0);
		Vector3f H_vel = H.slice<3,1>(3,0);
		Vector3f H_acc = H.slice<3,1>(6,0);
		Vector3f K_pos = Kfusion.slice<3,1>(0,0);
		Vector3f K_vel = Kfusion.slice<3,1>(3,0);
		Vector3f K_acc = Kfusion.slice<3,1>(6,0);


		augstate.aug_pos = augstate.aug_pos + K_pos*(mea_acc - H_pos*augstate.aug_pos);
		augstate.aug_vel = augstate.aug_vel + K_vel*(mea_acc - H_vel*augstate.aug_vel);
		augstate.aug_accel = augstate.aug_accel + K_acc*(mea_acc - H_acc*augstate.aug_accel);



		matrix::Matrix<float, 9, 9> I;
		I.setIdentity(); 

		P_aug = (I - Kfusion*H) * P_aug;

        Vector24f Kfusion_{};
        Kfusion_.slice<3,1>(4,0) = K_pos;
        Kfusion_.slice<3,1>(6,0) = K_vel;



	

		//ATTIVA LA FUSIONE

	   //measurementUpdate(Kfusion, _load_innov_var, _load_innov);
    //augstate.aug_accel(2) = augstate.aug_accel(2) + 9.8f;
	   

	//PUBLISHER DI DEBUG
	struct external_wrench_estimation_s wrench_estimation = {};


	wrench_estimation.timestamp = loadCell_Sample.time_us; 
	wrench_estimation.force_x = augstate.aug_accel(0);
	wrench_estimation.force_y = augstate.aug_pos(2);
	wrench_estimation.force_z = augstate.aug_accel(1);     
	wrench_estimation.torque_x =  augstate.aug_pos(1);                 
	wrench_estimation.torque_y = augstate.aug_accel(2);                 
	wrench_estimation.torque_z = augstate.aug_vel(2);                 

if (_wrench_pub == nullptr) {
    
    _wrench_pub = orb_advertise(ORB_ID(external_wrench_estimation), &wrench_estimation);
} else {

    orb_publish(ORB_ID(external_wrench_estimation), _wrench_pub, &wrench_estimation);
}

}




Vector3f Ekf:: retrieveAngularVelocity(){


    vehicle_angular_velocity_s vehicle_angular_velocity_data;
    Vector3f angular_velocity(0.0f, 0.0f, 0.0f);  // Inizializzazione a 0

    if (vehicle_angular_velocity_sub.update(&vehicle_angular_velocity_data)) {
        // Assegna i valori correttamente
        angular_velocity(0) = vehicle_angular_velocity_data.xyz[0];
        angular_velocity(1) = vehicle_angular_velocity_data.xyz[1];
        angular_velocity(2) = vehicle_angular_velocity_data.xyz[2];
    }

    return angular_velocity;
}


	

void Ekf::updateThrustBuffer(float thrust) {
    thrust_buffer.push_back(thrust);

    // Mantieni solo gli ultimi N campioni per introdurre il ritardo
    if (thrust_buffer.size() > thrust_delay_steps) {
        thrust_buffer.pop_front();  // Rimuove il valore più vecchio
    }
}



float Ekf::getDelayedThrust() {
    if (thrust_buffer.size() < thrust_delay_steps) {
        return thrust_buffer.front();  // Se il buffer non è ancora pieno, usa il valore più vecchio
    } else {
        return thrust_buffer[0];  // Prende il valore ritardato
    }
}



























































/*
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

		float delayed_thrust = getDelayedThrust();
		
		Vector3f thrust_delayed_body(0,0,delayed_thrust);
		Vector3f thrust_delayed_NED =  _state.quat_nominal.rotateVector(thrust_delayed_body);

		//float estim_force_z = abs(mass*accel_z) - abs(total_thrust + gravity_force);
		//float estim_force_z = abs(mass*accel_z) - abs(delayed_thrust + gravity_force);
		float estim_force_z = abs(mass*accel_z) - abs(thrust_delayed_NED(2) + gravity_force);


		return estim_force_z;

}*/



void Ekf::updateMeasBuffer(float mea_force_z_raw){

    // Aggiungi il nuovo valore al buffer
    meas_z_buffer.push_back(mea_force_z_raw);

    // Mantieni la dimensione del buffer entro la finestra
   if (meas_z_buffer.size() > static_cast<std::size_t>(window_size_meas_buffer)) {
        meas_z_buffer.pop_front();
    }
}




 

float Ekf::filterForceMeas() {

 if (meas_z_buffer.empty()) {
        throw std::runtime_error("Il buffer di meas_z è vuoto!");
    }

    float sum = 0.0f;

    // Somma i valori nel buffer
    for (const float value : meas_z_buffer) {
        sum += value;
    }

    // Restituisci la media
    return sum / meas_z_buffer.size();

	
}


void Ekf::updateAccelZBuffer(float accel_z) {
    // Aggiungi il nuovo valore al buffer
    accel_z_buffer.push_back(accel_z);

    // Mantieni la dimensione del buffer entro la finestra
   if (accel_z_buffer.size() > static_cast<std::size_t>(window_size_accel_buffer)) {
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





float Ekf::estimate_external_force_z(
    const float mass,                     // Massa del drone (kg)
    //const matrix::Dcmf &R_b,        // Matrice di rotazione (sistema corpo -> sistema inerziale)
    //const matrix::Vector3f &imu_accel, // Accelerazioni specifiche dall'IMU (sistema corpo)
    float thrust,                   // Spinta totale generata dai rotori (N)
    float dt,                       // Intervallo di tempo tra le iterazioni (secondi)
    float K1,                       // Guadagno del filtro (frequenza naturale)
    float K2,                       // Guadagno del filtro (smorzamento)
    float &r,                       // Stima attuale della forza esterna (passata come riferimento)
    float &r_dot,                    // Derivata della stima della forza (passata come riferimento)
	float accel_z
) {
    const float g = 9.81f; // Accelerazione gravitazionale (m/s^2)

	

    // 2. Calcolo della forza grezza lungo l'asse z
    // Questa è la forza "teorica" che include tutto ciò che non è spiegato dal modello del drone.
    float thrust_z = thrust; // Proiezione della spinta lungo l'asse z
    float f_z_raw = -mass * (accel_z) + thrust_z + mass * g;



	//PX4_INFO("PREDICTED ACCEL Z %.4f",static_cast<double>(accel_z_inertial + g));
	//PX4_INFO("PREDICTED FORCE RAW %.4f",static_cast<double>(f_z_raw));

    // 3. Calcolo della derivata della forza esterna stimata
    // Qui applichiamo un filtro di seconda ordine per stabilizzare la stima.
    float r_ddot = K1 * K2 * (f_z_raw - r) - K1 * r_dot;

    // 4. Aggiornamento della derivata della forza e della stima
    // Utilizziamo l'integrazione numerica per aggiornare la stima e la sua derivata.
    r_dot += r_ddot * dt; // Aggiorna la derivata della forza
    r += r_dot * dt;      // Aggiorna la forza stimata

	//PX4_INFO("PREDICTED FORCE FILTERED %.4f",static_cast<double>(r));

	//PX4_INFO("PREDICTED FORCE DIFFERENCE %.4f",static_cast<double>(f_z_raw - r));

	

    // 5. Ritorna la forza esterna stimata
    return -r;
}












