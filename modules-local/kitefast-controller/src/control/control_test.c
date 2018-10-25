#include "control_test.h"

#include <stdio.h>

#include "vec3.h"

void control_test(double dcm_g2b_c[]){

	Vec3 dcm_test = {dcm_g2b_c[0], dcm_g2b_c[1], dcm_g2b_c[2]};
	printf("msg from within control_test, with c_math, Here is  the first row of dcm_g2b_c. \n" );
	printf("dcm_g2b_c[0] = %f\n",dcm_g2b_c[0]);
	printf("dcm_g2b_c[1] = %f\n",dcm_g2b_c[1]);
	printf("dcm_g2b_c[2] = %f\n",dcm_g2b_c[2]);
	printf("msg from within control_test, with c_math, Here is Vec3 of dcm_g2b_c. \n" );
	printf("Vec3 - dcm_test[0] = %f\n",dcm_test.x);
	printf("Vec3 - dcm_test[1] = %f\n",dcm_test.y);
	printf("Vec3 - dcm_test[2] = %f\n",dcm_test.z);

}
