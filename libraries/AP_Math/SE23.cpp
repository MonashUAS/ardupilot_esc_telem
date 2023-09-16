//#include "SE23.h"
#include "AP_Math.h"

//SE23::SE23(const Matrix3f& rot, const Vector3f& x, const Vector3f& w, float alpha)
    //: _rot(rot), _x(x), _w(w), _alpha(alpha) {}

SE23 SE23::operator *(const SE23& rhs) {
    Matrix3f rot_result = _rot * rhs._rot;
    rot_result.normalize();
    Vector3f x_result = _x + _rot * rhs._x + _w*rhs._alpha;
    Vector3f w_result = _w + _rot * rhs._w;
    float alpha_result = _alpha + rhs._alpha;
    return(SE23(rot_result, x_result, w_result, alpha_result));
}
//Calculates the matrix exponential of SE23
SE23 SE23::exponential(const Vector3f &S, const Vector3f &x,const Vector3f &w, float alpha) {
    const float theta = S.length();
    Matrix3f input_matrix = Matrix3f::skew_symmetric(S); //example of using statics
    float A, B, C, D, E;
    if (theta > 0.0000001f) {
        A = sinF(theta) / theta;
        B = (1.0f - cosF(theta)) / (theta * theta);
        C = (1.0f - A) / (theta * theta);
        D = (theta - sinF(theta)) / (theta * theta * theta);
        E = (1.0f - 0.5f*theta*theta - cosF(theta)) / (theta * theta * theta*theta);
    } else {
        A = 1.0f;
        B = 0.5f;
        C = 1.0f / 6.0f;
        D = 1.0f / 24.0f;
        E = 1.0f / 120.0f;
    }
    //create Identity matrix 
    Matrix3f identity_3;  // Declare an object of Matrix3
    identity_3.a.x = 1; identity_3.a.y = 0; identity_3.a.z = 0;  // Set the values to create an identity matrix
    identity_3.b.x = 0; identity_3.b.y = 1; identity_3.b.z = 0;
    identity_3.c.x = 0; identity_3.c.y = 0; identity_3.c.z = 1;
    //Create intermeadiatries for results
    Matrix3f result_rot = identity_3 + input_matrix*A + input_matrix*input_matrix*B;
    Matrix3f V = identity_3 + input_matrix*B + input_matrix*input_matrix*C;
    Matrix3f V_2 = identity_3*0.5f + input_matrix*D + input_matrix*input_matrix*E;
    //Assign results
    return(SE23(result_rot, V*x + V_2*w*alpha, V*w, alpha)); 
}

//Inverse of the ZHat Matrix used in CINS 
SE23 SE23::inverse_ZHat(const SE23& ZHat){
    Matrix3f result_rot = ZHat._rot;
    Vector3f result_x = -ZHat._x + ZHat._w*ZHat._alpha;
    Vector3f result_w = -ZHat._w ;
    float result_alpha = -1.0f* ZHat._alpha;
    return(SE23(result_rot, result_x, result_w, result_alpha));
}


const Matrix3f& SE23::rot() {
    return _rot;
}

const Vector3f& SE23::x() {
    return _x;
}

const Vector3f& SE23::w() {
    return _w;
}

float SE23::alpha() {
    return _alpha;
}
