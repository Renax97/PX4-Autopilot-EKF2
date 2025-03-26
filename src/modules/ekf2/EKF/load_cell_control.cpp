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

	
		if (_load_cell_buffer->pop_first_older_than(_time_delayed_us, &loadCell_sample)) {

		updateLoadCell(loadCell_sample);
		}


      prev_vel(2) = _state.vel(2);
		
		
	}
}








void Ekf::updateLoadCell(const loadCellSample &loadCell_Sample){


    // Parametri del contatto elastico
    
    const float R_FORCE = 0.0001f; 
	const float FORCE_THRESHOLD = 0.05f;
	//const float bias_load_cell = - 0.0196;

   
    float mea_force_z_raw = -(loadCell_Sample.force(1));
	

    float alpha = 0.1f;
    float mea_force_z_filtered = alpha * mea_force_z_raw + (1.0f - alpha) * mea_force_z_filtered_old;
    mea_force_z_filtered_old = mea_force_z_filtered;

    //float mea_force_z_filtered = -(loadCell_Sample.force(1));


    /*Vector3f F_e_body (0, 0, mea_force_z_filtered);
    Vector3f F_e = F_e_body;*/
    //Vector3f F_e = _state.quat_nominal.rotateVector(F_e_body);


    

    
    if(mea_force_z_filtered < FORCE_THRESHOLD){

        v_parallel_z = _state.vel(2);

    }else{

        v_parallel_z = 0.0f;
        contact_happened = true;
    }




    _load_innov = _state.vel(2) - v_parallel_z;

  


    Vector24f H;
    H.setZero();
	
    H(6) = 1.0f;

    


     
    //const Vector24f state_vector_prev = getStatros2 run ros_gz_bridge parameter_bridge /world/default/model/x500_vision_0/joint/link2_link3_joint/sensor/force_torque_sensor/forcetorque@geometry_msgs/msg/WrenchStamped@gz.msgs.Wrenchce_z_raw;

    

    //float load_innov_var_x = (Hx.transpose() * P * Hx)(0, 0) + R_FORCE;
    //float load_innov_var_y = (Hy.transpose() * P * Hy)(0, 0) + R_FORCE;
     _load_innov_var = (H.transpose() * P * H)(0, 0) + R_FORCE;


		//Vector24f Kfusionx = P * Hx / load_innov_var_x;
        //Vector24f Kfusiony = P * Hy / load_innov_var_y;
        Vector24f Kfusion = P * H / _load_innov_var;
       




        /*
        float load_cell_z_test_ratio = sq(load_innov_z  / (sq(5.0f) * load_innov_var_z));*/

		// if the innovation consistency check fails then don't fuse the sample
		
        //if (load_cell_z_test_ratio <= 1.0f) {


        if(contact_happened){
            time_of_contact += _dt_ekf_avg;
        }
     
        
        if(mea_force_z_filtered > FORCE_THRESHOLD && time_of_contact < 0.1f){

        measurementUpdate(Kfusion, _load_innov_var, _load_innov);

    

    }else{
        v_parallel_z = _state.vel(2);

    }
    
//}
        

     
         





	//PUBLISHER DI DEBUG
	struct external_wrench_estimation_s wrench_estimation = {};

	wrench_estimation.timestamp = loadCell_Sample.time_us; // Tempo corrente
	wrench_estimation.force_x = 0.0f;                  // Forza su X (fissata a 0)
	wrench_estimation.force_y = _load_innov_var;                  // Forza su Y (fissata a 0)
	wrench_estimation.force_z = _load_innov;     // Forza stimata su Z
	wrench_estimation.torque_x = Kfusion(6);                 // Momento torcente su X
	wrench_estimation.torque_y = v_parallel_z;                 // Momento torcente su Y
	wrench_estimation.torque_z = mea_force_z_filtered;                 // Momento torcente su Z

if (_wrench_pub == nullptr) {
    
    _wrench_pub = orb_advertise(ORB_ID(external_wrench_estimation), &wrench_estimation);
} else {

    orb_publish(ORB_ID(external_wrench_estimation), _wrench_pub, &wrench_estimation);
}





}































































 








