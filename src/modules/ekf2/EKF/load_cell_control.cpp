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


		


     

		if (_load_cell_buffer->pop_first_older_than(_time_delayed_us, &loadCell_sample)) {

		updateLoadCell(loadCell_sample);
		}


      prev_augstate_vel = _state.vel;
		
		
	}
}








void Ekf::updateLoadCell(const loadCellSample &loadCell_Sample){


    // Parametri del contatto elastico
    
    const float R_FORCE = 0.4f; 
    //const float mass = _params.mass;
	//const float mass = 2.081f;
	//const float mass_drone = 2.0643f;
	//const float mass_arm = 0.017f;
	//const float mass = mass_drone + mass_arm; 
	const float FORCE_THRESHOLD = 0.00000001f;
	//const float bias_load_cell = - 0.0196;

    //MISURA ACCELERAZIONE Z
	float mea_force_x_raw = -(loadCell_Sample.force(2));
    float mea_force_y_raw = (loadCell_Sample.force(0));
    float mea_force_z_raw = -(loadCell_Sample.force(1));
	//updateMeasBuffer(mea_force_z_raw);
	//float mea_force_z_filtered = filterForceMeas();
	//float mea_acc = mea_force_z_filtered/mass;

    float alpha = 0.05;
	float mea_force_x_filtered = alpha * mea_force_x_raw + (1.0f - alpha) * mea_force_x_filtered_old;
    mea_force_x_filtered_old = mea_force_x_filtered;
    float mea_force_y_filtered = alpha * mea_force_y_raw + (1.0f - alpha) * mea_force_y_filtered_old;
    mea_force_y_filtered_old = mea_force_y_filtered;
    float mea_force_z_filtered = alpha * mea_force_z_raw + (1.0f - alpha) * mea_force_z_filtered_old;
    mea_force_z_filtered_old = mea_force_z_filtered;

    Vector3f F_e_body (mea_force_x_filtered, mea_force_y_filtered, mea_force_z_filtered);
    Vector3f F_e = _state.quat_nominal.rotateVector(F_e_body);


    //Vector3f F_e (mea_force_x_filtered, mea_force_y_filtered, mea_force_z_filtered);
    Vector3f v_parallel;

    float dot_vF = prev_augstate_vel(0) * F_e(0) + prev_augstate_vel(1) * F_e(1) + prev_augstate_vel(2) * F_e(2);

    float norm_F_squared = F_e(0)* F_e(0) + F_e(1) * F_e(1) + F_e(2) * F_e(2);

    if(norm_F_squared < FORCE_THRESHOLD){

        v_parallel = prev_augstate_vel;

    }else{

        float projection_coefficient = dot_vF / norm_F_squared;
         for (int i = 0; i < 3; i++) {
        v_parallel(i) = prev_augstate_vel(i) - projection_coefficient * F_e(i);
    }
    }

    float load_innov_x = _state.vel(0) - v_parallel(0);
    float load_innov_y = _state.vel(1) - v_parallel(1);
    float load_innov_z = _state.vel(2) - v_parallel(2);



    
	
    Vector24f Hx;
    Vector24f Hy;
    Vector24f Hz;
	
	Hx(4) = 1.0f;  // Aggiorna solo v_z
	Hy(5) = 1.0f;
    Hz(6) = 1.0f;


     
    //const Vector24f state_vector_prev = getStateAtFusionHorizonAsVector();

    //_load_innov = H*state_vector_prev - mea_force_z_raw;

    

    float load_innov_var_x = (Hx.transpose() * P * Hx)(0, 0) + R_FORCE;
    float load_innov_var_y = (Hy.transpose() * P * Hy)(0, 0) + R_FORCE;
    float load_innov_var_z = (Hz.transpose() * P * Hz)(0, 0) + R_FORCE;


		Vector24f Kfusionx = P * Hx / load_innov_var_x;
        Vector24f Kfusiony = P * Hy / load_innov_var_y;
        Vector24f Kfusionz = P * Hz / load_innov_var_z;




      
        measurementUpdate(Kfusionx, load_innov_var_x, load_innov_x);
        measurementUpdate(Kfusiony, load_innov_var_y, load_innov_y);
        measurementUpdate(Kfusionz, load_innov_var_z, load_innov_z);

     
         





	//PUBLISHER DI DEBUG
	struct external_wrench_estimation_s wrench_estimation = {};

	wrench_estimation.timestamp = loadCell_Sample.time_us; // Tempo corrente
	wrench_estimation.force_x = load_innov_x;                  // Forza su X (fissata a 0)
	wrench_estimation.force_y = load_innov_y;                  // Forza su Y (fissata a 0)
	wrench_estimation.force_z = load_innov_z;     // Forza stimata su Z
	wrench_estimation.torque_x = 0.0f;                 // Momento torcente su X
	wrench_estimation.torque_y =mea_force_z_filtered;                 // Momento torcente su Y
	wrench_estimation.torque_z = 0.0f;                 // Momento torcente su Z

if (_wrench_pub == nullptr) {
    
    _wrench_pub = orb_advertise(ORB_ID(external_wrench_estimation), &wrench_estimation);
} else {

    orb_publish(ORB_ID(external_wrench_estimation), _wrench_pub, &wrench_estimation);
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












