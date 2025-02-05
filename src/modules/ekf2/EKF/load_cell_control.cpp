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
		//const float FORCE_THRESHOLD = 1.0f;
		const float bias_load_cell = - 0.0196;

		
		
		



		//CALCOLO DEL THRUST
	    float total_thrust = compute_thrust_z();
		/*Vector3f thrust_body(0.0f, 0.0f, total_thrust);
		Vector3f thrust_NED = _state.quat_nominal.rotateVector(thrust_body);
		total_thrust = thrust_NED(2);*/





		/*float dt = _dt_ekf_avg;
		float K1 = 100.0f;     // Guadagno del filtro
		float K2 = 10.0f;      // Guadagno di smorzamento
		// Variabili per mantenere lo stato del filtro
		float r = 0.0f;       // Stima iniziale della forza esterna
		float r_dot = 0.0f;   // Derivata iniziale della forza esterna
		float f_z_ext = estimate_external_force_z(mass, total_thrust,dt, K1, K2, r, r_dot,accel_z);*/


		//PREDIZIONE ACCELERAZIONE Z
		float estimated_force_z = predict_fz(mass,total_thrust,accel_z);
		
		float pred_acc = estimated_force_z/mass;


		//MISURA ACCELERAZIONE Z
	    float mea_force_z_raw = -(loadCell_Sample.force(1) - bias_load_cell);

		updateMeasBuffer(mea_force_z_raw);

		float mea_force_z_filtered = filterForceMeas();


		//mea_force_z_filtered = alpha_load_cell_filter * mea_force_z + (1.0f - alpha_load_cell_filter) * mea_force_z_filtered;


		float mea_acc = mea_force_z_filtered/mass;

		//pred_acc = _state.vel(2);

		//mea_acc = ((mea_acc * _dt_ekf_avg) + _state.vel(2));


		//CALCOLO INNOVAZIONE
        _load_innov = pred_acc - mea_acc;
		//PX4_INFO("INNOVAZIONE: %.4f", static_cast<double>(_load_innov));
		

		//CALCOLO VARIANZA INNOVAZIONE E GUADAGNO DI KALMAN
		 const float H_vz = 0.01f;
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

	   measurementUpdate(Kfusion, _load_innov_var, _load_innov);

	   /*if (fabs(loadCell_Sample.force(1)) > FORCE_THRESHOLD*100) {
        measurementUpdate(Kfusion, _load_innov_var, _load_innov);
		}*/



	//const float gravity_force = mass*CONSTANTS_ONE_G;

	float thrust_delayed_debug = getDelayedThrust();

	Vector3f thrust_delayed_body(0,0,thrust_delayed_debug);
	Vector3f thrust_delayed_NED =  _state.quat_nominal.rotateVector(thrust_delayed_body);


	//PUBLISHER DI DEBUG
	struct external_wrench_estimation_s wrench_estimation = {};

	wrench_estimation.timestamp = loadCell_Sample.time_us; // Tempo corrente
	wrench_estimation.force_x = mea_force_z_filtered;                  // Forza su X (fissata a 0)
	wrench_estimation.force_y = total_thrust;                  // Forza su Y (fissata a 0)
	wrench_estimation.force_z = estimated_force_z;     // Forza stimata su Z
	wrench_estimation.torque_x = _load_innov;                 // Momento torcente su X
	wrench_estimation.torque_y = accel_z*mass;                 // Momento torcente su Y
	wrench_estimation.torque_z = thrust_delayed_NED(2);                 // Momento torcente su Z

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

		total_thrust = - total_thrust;

		updateThrustBuffer(total_thrust);

    } else {
        PX4_WARN("Nessun dato disponibile da actuator_outputs");
    }


	return total_thrust;

}



float Ekf::predict_fz(const float mass, float total_thrust, const float accel_z){


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

}



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












