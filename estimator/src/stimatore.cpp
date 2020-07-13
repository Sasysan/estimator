#include "stimatore.h"
#include<math.h>

using namespace std;
using namespace Eigen;

//---Helper functions 
Matrix3d QuatToMat(Vector4d Quat){
    Matrix3d Rot;
    float s = Quat[0];
    float x = Quat[1];
    float y = Quat[2];
    float z = Quat[3];
    Rot << 1-2*(y*y+z*z),2*(x*y-s*z),2*(x*z+s*y),
    2*(x*y+s*z),1-2*(x*x+z*z),2*(y*z-s*x),
    2*(x*z-s*y),2*(y*z+s*x),1-2*(x*x+y*y);
    return Rot;
}

Vector4d rot2quat(Matrix3d R){
    float m00, m01, m02, m10, m11, m12, m20, m21, m22;

    m00 = R(0,0);
    m01 = R(0,1);
    m02 = R(0,2);
    m10 = R(1,0);
    m11 = R(1,1);
    m12 = R(1,2);
    m20 = R(2,0);
    m21 = R(2,1);
    m22 = R(2,2);

    float tr = m00 + m11 + m22;
    float qw, qx, qy, qz, S;
    Vector4d quat;

    if (tr > 0) { 
        S = sqrt(tr+1.0) * 2; // S=4*qw 
        qw = 0.25 * S;
        qx = (m21 - m12) / S;
        qy = (m02 - m20) / S; 
        qz = (m10 - m01) / S; 
    } else if ((m00 > m11)&(m00 > m22)) { 
        S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx 
        qw = (m21 - m12) / S;
        qx = 0.25 * S;
        qy = (m01 + m10) / S; 
        qz = (m02 + m20) / S; 
    } else if (m11 > m22) { 
        S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
        qw = (m02 - m20) / S;
        qx = (m01 + m10) / S; 
        qy = 0.25 * S;
        qz = (m12 + m21) / S; 
    } else { 
        S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
        qw = (m10 - m01) / S;
        qx = (m02 + m20) / S;
        qy = (m12 + m21) / S;
        qz = 0.25 * S;
    }

    quat << qw, qx, qy, qz;
    return quat;
}
	
	
Vector3d R2XYZ(Matrix3d R) {
    double phi=0.0, theta=0.0, psi=0.0;
    Vector3d XYZ = Vector3d::Zero();
    
    theta = asin(R(0,2));
    
    if(fabsf(cos(theta))>pow(10.0,-10.0))
    {
        phi=atan2(-R(1,2)/cos(theta), R(2,2)/cos(theta));
        psi=atan2(-R(0,1)/cos(theta), R(0,0)/cos(theta));
    }
    else
    {
        if(fabsf(theta-M_PI/2.0)<pow(10.0,-5.0))
        {
            psi = 0.0;
            phi = atan2(R(1,0), R(2,0));
            theta = M_PI/2.0;
        }
        else
        {
            psi = 0.0;
            phi = atan2(-R(1,0), R(2,0));
            theta = -M_PI/2.0;
        }
    }
    
    XYZ << phi,theta,psi;
    return XYZ;
}

Matrix3d XYZ2R(Vector3d angles) {
    
    Matrix3d R = Matrix3d::Zero(); 
    Matrix3d R1 = Matrix3d::Zero(); 
    Matrix3d R2 = Matrix3d::Zero(); 
    Matrix3d R3 = Matrix3d::Zero();

    float cos_phi = cos(angles[0]);
    float sin_phi = sin(angles[0]);
    float cos_theta = cos(angles[1]);
    float sin_theta = sin(angles[1]);
    float cos_psi = cos(angles[2]);
    float sin_psi = sin(angles[2]);

    R1  << 1, 0      , 0, 
                0, cos_phi, -sin_phi, 
                0, sin_phi, cos_phi;

    R2  << cos_theta , 0, sin_theta, 
                0        , 1, 0       , 
                -sin_theta, 0, cos_theta;

    R3  << cos_psi, -sin_psi, 0, 
                sin_psi, cos_psi , 0,
                0      , 0       , 1;

    R = R1*R2*R3;

    return R;
}
// Fine funzioni aiuto




STIMATORE::STIMATORE() {
    _odom_sub=_nh.subscribe ("/iris_smc/odometry",0,&STIMATORE::Odomcallback,this);
    _stima_pub=_nh.advertise<std_msgs::Float32MultiArray>("/stima",0);
    _force_thrust_sub=_nh.subscribe("/iris_smc/cmd/acc_thr",0,&STIMATORE::ForceThrustcallback,this);
    _mass=1.53;
    _gravity=9.81;
    _Ib<<0.0347563,0,0,0, 0.0458929,0,0, 0.0977;
    _stima=Matrix<double,6,1>::Zero();
    K1.diagonal()<<1,1,1,0.5,1,0.5;
    K2.diagonal()<<0.25,0.25,0.25,0.125,0.25,0.125;
}

void STIMATORE::Odomcallback(const nav_msgs::Odometry odometry_msg){
    _odometr= odometry_msg;
    _eta = R2XYZ( QuatToMat ( Vector4d( _odometr.pose.pose.orientation.w,  _odometr.pose.pose.orientation.x, _odometr.pose.pose.orientation.y, _odometr.pose.pose.orientation.z ) ) );
    _eta_dot<<_odometr.twist.twist.angular.x,_odometr.twist.twist.angular.y,_odometr.twist.twist.angular.z;
    _vel<<_odometr.twist.twist.linear.x,_odometr.twist.twist.linear.y,_odometr.twist.twist.linear.z;
}

void STIMATORE::ForceThrustcallback(const std_msgs::Float32MultiArray force_thrust){
    _force=-1*force_thrust.data[3];
    _tau_b<<force_thrust.data[0],force_thrust.data[1],force_thrust.data[2];
    }


void STIMATORE::momentum_dot(){
    //_eta=Vector3d::Zero();   
    //_eta_dot=Vector3d::Zero();
 //Vector4f quaternion;
 //quaternion<<_odometr.pose.pose.orientation.w,  _odometr.pose.pose.orientation.x, _odometr.pose.pose.orientation.y, _odometr.pose.pose.orientation.z;
 //_eta = R2XYZ( QuatToMat ( Vector4d( _odometr.pose.pose.orientation.w,  _odometr.pose.pose.orientation.x, _odometr.pose.pose.orientation.y, _odometr.pose.pose.orientation.z ) ) );
 //Eigen::Quaterniond q(quaternion(0), quaternion(1), quaternion(2), quaternion(3));
 
 //_eta= q.toRotationMatrix().eulerAngles(0, 1, 2);
 //_eta_dot<<_odometr.twist.twist.angular.x,_odometr.twist.twist.angular.y,_odometr.twist.twist.angular.z;

    _Jacobian(0,0)=1;
    _Jacobian(0,1)=0;
    _Jacobian(0,2)=-sin(_eta(2));
    _Jacobian(1,0)=0;
    _Jacobian(1,1)=cos(_eta(1));
    _Jacobian(1,2)=cos(_eta(2))*sin(_eta(1));
    _Jacobian(2,0)=0;
    _Jacobian(2,1)=-sin(_eta(1));
    _Jacobian(2,2)=cos(_eta(2))*cos(_eta(1));

    Matrix3d Jacobian_dot;
    Jacobian_dot(0,0)=0;
    Jacobian_dot(0,1)=0;
    Jacobian_dot(0,2)=-cos(_eta(2))*_eta_dot(2);
    Jacobian_dot(1,0)=0;
    Jacobian_dot(1,1)=-sin(_eta(1))*_eta_dot(1);
    Jacobian_dot(1,2)=(-sin(_eta(2))*sin(_eta(1))*_eta_dot(2))+(cos(_eta(1))*cos(_eta(2))*_eta_dot(1));
    Jacobian_dot(2,0)=0;
    Jacobian_dot(2,1)=-cos(_eta(1))*_eta_dot(1);
    Jacobian_dot(2,2)=(-sin(_eta(2))*cos(_eta(1))*_eta_dot(2))-(sin(_eta(1))*cos(_eta(2))*_eta_dot(1));

    Vector3d omega;
    omega=_Jacobian*_eta_dot;

    Matrix3d S_c;
    S_c(0,0)=0;
    S_c(0,1)=-omega(3);
    S_c(0,2)=omega(2);
    S_c(1,0)=omega(3);
    S_c(1,1)=0;
    S_c(1,2)=-omega(1);
    S_c(2,0)=-omega(2);
    S_c(2,1)=omega(1);
    S_c(2,2)=0;

    Matrix3d C;
    C=(_Jacobian.transpose()*S_c*_Ib*_Jacobian)+(_Jacobian.transpose()*_Ib*Jacobian_dot);
   
    Vector3d ez(Eigen::Vector3d::UnitZ());
    
    Vector3d Rb_col3;
    //Rb_col3=QuatToMat ( Vector4d( _odometr.pose.pose.orientation.w,  _odometr.pose.pose.orientation.x, _odometr.pose.pose.orientation.y, _odometr.pose.pose.orientation.z ) )*ez ;
    Rb_col3=XYZ2R(_eta)*ez ;
    //Rb_col3<<(cos(_eta(1))*sin(_eta(2))*cos(_eta(3)))+(sin(_eta(1))*sin(_eta(3))),(cos(_eta(1))*sin(_eta(2))*sin(_eta(3)))-(sin(_eta(1))*cos(_eta(3))),cos(_eta(1))*cos(_eta(2));
    
    Vector3d force_temp;
    force_temp=(-_force*Rb_col3)+(_mass*_gravity*ez);
    
    Vector3d torque_temp;
    torque_temp=(_Jacobian.transpose()*_tau_b)+(C.transpose()*_eta_dot);
    
    _momentum_dot <<force_temp,torque_temp;
    //cout<<"Coriolis="<<C.transpose()<<endl;
    //cout<<"Jacobian="<<_Jacobian.transpose()<<endl;
    //cout<<"Momentum_dot="<<_momentum_dot<<endl; 

};

void STIMATORE::momentum(){
  
   //_vel<<_odometr.twist.twist.linear.x,_odometr.twist.twist.linear.y,_odometr.twist.twist.linear.z;

    Matrix3d M;
    M=_Jacobian.transpose()*_Ib*_Jacobian;

    Matrix<double,6,6> momentum_temp;
    momentum_temp<<_mass*Matrix3d::Identity(),Matrix3d::Zero(),
                    Matrix3d::Zero(),M;
    
    Matrix<double,6,1> force_torque;
    force_torque<<_vel,_eta_dot;
    
    _momentum=momentum_temp*force_torque;
   //cout<<"Momentum="<<_momentum<<endl; 
}

void STIMATORE::stima(){
  Matrix<double,6,1> stima_dot;
  Matrix<double,6,1> stima_onetemp;
  Matrix<double,6,1> stima_twotemp;
  Matrix<double,6,1> stima_threetemp;
  Matrix<double,6,1> stima_fourtemp;
  stima_dot=Matrix<double,6,1>::Zero();
  stima_onetemp=Matrix<double,6,1>::Zero();
  stima_twotemp=Matrix<double,6,1>::Zero();
  stima_threetemp=Matrix<double,6,1>::Zero();
  stima_fourtemp=Matrix<double,6,1>::Zero();

  ros::Rate rate(100);
  std_msgs::Float32MultiArray stima_msg;
  stima_msg.data.resize(6);
 
  while(ros::ok()){
   momentum_dot();
   momentum();
   
   stima_dot=_stima+_momentum_dot;
   //cout<<"stima_dot="<<stima_dot<<endl;
   stima_onetemp=stima_onetemp+(stima_dot*(1.0/100.0));
   //cout<<"stima_onetemp="<<stima_onetemp<<endl;
   stima_twotemp=K2*(_momentum-stima_onetemp);
   //cout<<"stima_twotemp="<<stima_twotemp<<endl;
   stima_threetemp=stima_twotemp-_stima;
   //cout<<"stima_threetemp="<<stima_threetemp<<endl;
   //stima_fourtemp=stima_fourtemp+(stima_threetemp*(1.0/100.0));
   _stima=K1*(_stima+(stima_threetemp*(1.0/100.0)));
   //cout<<"stima="<<_stima<<endl;  
    
   
   
   
    for(int i=0;i<6;i++){
        stima_msg.data[i]=_stima(i);
        cout<<"stima("<<i<<"): "<<_stima(i)<<endl;
    }
   
   /*
    for(int i=0; i<stima_msg.data.size(); i++ ) {
            stima_msg.data[i] = _stima(i); 
        }
*/  

      
        _stima_pub.publish( stima_msg );


    rate.sleep();
    ros::spinOnce();
  }
      
    
}

void STIMATORE::run(){
    stima();
    ros::spin();
}

int main(int argc,char** argv){
ros::init(argc,argv,"stimatore");
STIMATORE st;
st.run();
return 0;
}