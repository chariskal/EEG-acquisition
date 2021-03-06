#include <pa10_remote/pa10remote.h>
#define PI acos(-1)

using namespace Eigen;


MatrixXd pa10FKDHModified (VectorXd theta){

     // DH Parameters

     VectorXd alpha(7);
     alpha << 0.0, -PI/2.0, PI/2.0, -PI/2.0, PI/2.0, -PI/2.0, PI/2.0;

     VectorXd a(7);
     a << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

     VectorXd d(7);
     d << 0.0, 0.0, 0.450, 0.0, 0.480, 0.0, 0.18;

     //FK Calculation

     MatrixXd A0(4,4); 
              A0 << cos(theta(0)), -sin(theta(0)), 0.0, a(0),
           	    sin(theta(0))*cos(alpha(0)), cos(theta(0))*cos(alpha(0)), -sin(alpha(0)), -sin(alpha(0))*d(0),
           	    sin(theta(0))*sin(alpha(0)), cos(theta(0))*sin(alpha(0)),  cos(alpha(0)), cos(alpha(0))*d(0),
           	    0.0, 0.0, 0.0, 1.0;

     Matrix4d A1(4,4);
              A1 << cos(theta(1)), -sin(theta(1)), 0.0, a(1),
           	    sin(theta(1))*cos(alpha(1)), cos(theta(1))*cos(alpha(1)), -sin(alpha(1)), -sin(alpha(1))*d(1),
           	    sin(theta(1))*sin(alpha(1)), cos(theta(1))*sin(alpha(1)),  cos(alpha(1)), cos(alpha(1))*d(1),
           	    0.0, 0.0, 0.0, 1.0;
  
     Matrix4d A2(4,4);
              A2 << cos(theta(2)), -sin(theta(2)), 0.0, a(2),
           	    sin(theta(2))*cos(alpha(2)), cos(theta(2))*cos(alpha(2)), -sin(alpha(2)), -sin(alpha(2))*d(2),
           	    sin(theta(2))*sin(alpha(2)), cos(theta(2))*sin(alpha(2)),  cos(alpha(2)), cos(alpha(2))*d(2),
           	    0.0, 0.0, 0.0, 1.0;

     Matrix4d A3(4,4);
              A3 << cos(theta(3)), -sin(theta(3)), 0.0, a(3),
           	    sin(theta(3))*cos(alpha(3)), cos(theta(3))*cos(alpha(3)), -sin(alpha(3)), -sin(alpha(3))*d(3),
           	    sin(theta(3))*sin(alpha(3)), cos(theta(3))*sin(alpha(3)),  cos(alpha(3)), cos(alpha(3))*d(3),
           	    0.0, 0.0, 0.0, 1.0;

     Matrix4d A4(4,4);
              A4 << cos(theta(4)), -sin(theta(4)), 0.0, a(4),
           	    sin(theta(4))*cos(alpha(4)), cos(theta(4))*cos(alpha(4)), -sin(alpha(4)), -sin(alpha(4))*d(4),
           	    sin(theta(4))*sin(alpha(4)), cos(theta(4))*sin(alpha(4)),  cos(alpha(4)), cos(alpha(4))*d(4),
           	    0.0, 0.0, 0.0, 1.0;

     Matrix4d A5(4,4);
              A5 << cos(theta(5)), -sin(theta(5)), 0.0, a(5),
           	    sin(theta(5))*cos(alpha(5)), cos(theta(5))*cos(alpha(5)), -sin(alpha(5)), -sin(alpha(5))*d(5),
           	    sin(theta(5))*sin(alpha(5)), cos(theta(5))*sin(alpha(5)),  cos(alpha(5)), cos(alpha(5))*d(5),
           	    0.0, 0.0, 0.0, 1.0;

     Matrix4d A6(4,4);
              A6 << cos(theta(6)), -sin(theta(6)), 0.0, a(6),
           	    sin(theta(6))*cos(alpha(6)), cos(theta(6))*cos(alpha(6)), -sin(alpha(6)), -sin(alpha(6))*d(6),
           	    sin(theta(6))*sin(alpha(6)), cos(theta(6))*sin(alpha(6)),  cos(alpha(6)), cos(alpha(6))*d(6),
           	    0.0, 0.0, 0.0, 1.0;

     return A0*A1*A2*A3*A4*A5*A6;
           
 
}


MatrixXd pa10Jacobian(VectorXd q){

     MatrixXd Jp(3,7);
     MatrixXd Jf(3,7);     
     MatrixXd Jac(6,7);

     double s1 = sin(q(0));
     double c1 = cos(q(0));
     double s2 = sin(q(1));
     double c2 = cos(q(1));
     double s3 = sin(q(2));
     double c3 = cos(q(2));
     double s4 = sin(q(3));
     double c4 = cos(q(3));
     double s5 = sin(q(4));
     double c5 = cos(q(4));
     double s6 = sin(q(5));
     double c6 = cos(q(5));
     double s7 = sin(q(6));
     double c7 = cos(q(6));

     double d7=0.11;

     Jp(0,0) = -0.45*s1*s2+0.48*(-c4*s1*s2+(-c2*c3*s1-c1*s3)*s4)+(0.07+d7)*(-c6*(c4*s1*s2-(-c2*c3*s1-c1*s3)*s4)+(c5*(c4*(-c2*c3*s1-c1*s3)+s1*s2*s4)+(-c1*c3+c2*s1*s3)*s5)*s6);

     Jp(0,1) = 0.45*c1*c2+0.48*(c1*c2*c4-c1*c3*s2*s4)+(0.07+d7)*(-c6*(-c1*c2*c4+c1*c3*s2*s4)+(c5*(-c1*c3*c4*s2-c1*c2*s4)+c1*s2*s3*s5)*s6);

     Jp(0,2) = 0.48*(-c3*s1-c1*c2*s3)*s4+(0.07+d7)*(c6*(-c3*s1-c1*c2*s3)*s4+(c4*c5*(-c3*s1-c1*c2*s3)+(-c1*c2*c3+s1*s3)*s5)*s6);

     Jp(0,3) = 0.48*(c4*(c1*c2*c3-s1*s3)-c1*s2*s4)+(0.07+d7)*(-c6*(-c4*(c1*c2*c3-s1*s3)+c1*s2*s4)+c5*(-c1*c4*s2-(c1*c2*c3-s1*s3)*s4)*s6);

     Jp(0,4) = (0.07+d7)*(c5*(-c3*s1-c1*c2*s3)-(c4*(c1*c2*c3-s1*s3)-c1*s2*s4)*s5)*s6;

     Jp(0,5) = (0.07+d7)*(c6*(c5*(c4*(c1*c2*c3-s1*s3)-c1*s2*s4)+(-c3*s1-c1*c2*s3)*s5)+(-c1*c4*s2-(c1*c2*c3-s1*s3)*s4)*s6);

     Jp(0,6) = 0.0;

     Jp(1,0) = 0.45*c1*s2+0.48*(c1*c4*s2+(c1*c2*c3-s1*s3)*s4)+(0.07+d7)*(-c6*(-c1*c4*s2-(c1*c2*c3-s1*s3)*s4)+(c5*(c4*(c1*c2*c3-s1*s3)-c1*s2*s4)+(-c3*s1-c1*c2*s3)*s5)*s6);

     Jp(1,1) = 0.45*c2*s1+0.48*(c2*c4*s1-c3*s1*s2*s4)+(0.07+d7)*(-c6*(-c2*c4*s1+c3*s1*s2*s4)+(c5*(-c3*c4*s1*s2-c2*s1*s4)+s1*s2*s3*s5)*s6);

     Jp(1,2) = 0.48*(c1*c3-c2*s1*s3)*s4+(0.07+d7)*(c6*(c1*c3-c2*s1*s3)*s4+(c4*c5*(c1*c3-c2*s1*s3)+(-c2*c3*s1-c1*s3)*s5)*s6);

     Jp(1,3) = 0.48*(c4*(c2*c3*s1+c1*s3)-s1*s2*s4)+(0.07+d7)*(-c6*(-c4*(c2*c3*s1+c1*s3)+s1*s2*s4)+c5*(-c4*s1*s2-(c2*c3*s1+c1*s3)*s4)*s6);

     Jp(1,4) = (0.07+d7)*(c5*(c1*c3-c2*s1*s3)-(c4*(c2*c3*s1+c1*s3)-s1*s2*s4)*s5)*s6;

     Jp(1,5) = (0.07+d7)*(c6*(c5*(c4*(c2*c3*s1+c1*s3)-s1*s2*s4)+(c1*c3-c2*s1*s3)*s5)+(-c4*s1*s2-(c2*c3*s1+c1*s3)*s4)*s6);

     Jp(1,6) = 0.0;

     Jp(2,0) = 0.0;

     Jp(2,1) = -0.45*s2+0.48*(-c4*s2-c2*c3*s4)+(0.07+d7)*(-c6*(c4*s2+c2*c3*s4)+(c5*(-c2*c3*c4+s2*s4)+c2*s3*s5)*s6);

     Jp(2,2) = 0.48*s2*s3*s4+(0.07+d7)*(c6*s2*s3*s4+(c4*c5*s2*s3+c3*s2*s5)*s6);

     Jp(2,3) = 0.48*(-c3*c4*s2-c2*s4)+(0.07+d7)*(-c6*(c3*c4*s2+c2*s4)+c5*(-c2*c4+c3*s2*s4)*s6);

     Jp(2,4) = (0.07+d7)*(c5*s2*s3-(-c3*c4*s2-c2*s4)*s5)*s6;

     Jp(2,5) = (0.07+d7)*(c6*(c5*(-c3*c4*s2-c2*s4)+s2*s3*s5)+(-c2*c4+c3*s2*s4)*s6);

     Jp(2,6) = 0.0;


     Jf(0,0) = 0.0;

     Jf(0,1) = -s1;

     Jf(0,2) = c1*s2;

     Jf(0,3) = -c3*s1-c1*c2*s3;

     Jf(0,4) = c1*c4*s2+(c1*c2*c3-s1*s3)*s4;

     Jf(0,5) = c5*(-c3*s1-c1*c2*s3)-(c4*(c1*c2*c3-s1*s3)-c1*s2*s4)*s5;

     Jf(0,6) = -c6*(-c1*c4*s2-(c1*c2*c3-s1*s3)*s4)+(c5*(c4*(c1*c2*c3-s1*s3)-c1*s2*s4)+(-c3*s1-c1*c2*s3)*s5)*s6;

     Jf(1,0) = 0.0;

     Jf(1,1) = c1;

     Jf(1,2) = s1*s2;

     Jf(1,3) = c1*c3-c2*s1*s3;

     Jf(1,4) = c4*s1*s2+(c2*c3*s1+c1*s3)*s4;

     Jf(1,5) = c5*(c1*c3-c2*s1*s3)-(c4*(c2*c3*s1+c1*s3)-s1*s2*s4)*s5;

     Jf(1,6) = -c6*(-c4*s1*s2-(c2*c3*s1+c1*s3)*s4)+(c5*(c4*(c2*c3*s1+c1*s3)-s1*s2*s4)+(c1*c3-c2*s1*s3)*s5)*s6;

     Jf(2,0) = 1.0;

     Jf(2,1) = 0.0;

     Jf(2,2) = c2;

     Jf(2,3) = s2*s3;

     Jf(2,4) = c2*c4-c3*s2*s4;

     Jf(2,5) = c5*s2*s3-(-c3*c4*s2-c2*s4)*s5;

     Jf(2,6) = -c6*(-c2*c4+c3*s2*s4)+(c5*(-c3*c4*s2-c2*s4)+s2*s3*s5)*s6;
     

     Jac.block<3,7>(0,0) = Jp;
     Jac.block<3,7>(3,0) = Jf;

     return Jac;

}




