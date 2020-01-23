#include "Bone_Animation.h"


Bone_Animation::Bone_Animation()
{
}


Bone_Animation::~Bone_Animation()
{
}

void Bone_Animation::init()
{
	root_position = { 2.0f,1.0f,2.0f };
	yellow_position = { 2.0f,3.0f,2.0f };
	pink_position = { 2.0f,6.5f,2.0f };
	blue_position = { 2.0f,9.0f,2.0f };

	scale_vector =
	{
		{ 1.0f,1.0f,1.0f },
		{ 0.5f,4.f,0.5f },
		{ 0.5f,3.0f,0.5f },
		{ 0.5f,2.0f,0.5f }
	};

	rotation_degree_vector =
	{
		{ 0.0f,0.0f,0.0f },
		{ 0.0f,0.0f,0.0f },
		{ 0.0f,0.0f,0.0f },
		{ 0.0f,0.0f,0.0f }
	};

	colors =
	{
		{ 0.7f,0.0f,0.0f,1.0f },
		{ 0.7f,0.7f,0.0f,1.0f },
		{ 0.7f,0.0f,0.7f,1.0f },
		{ 0.0f,0.7f,0.7f,1.0f },
		{ 0.2f,0.6f,0.3f,1.0f }
	};
	reset();
}

void Bone_Animation::update(float delta_time)
{
	m_target_mat = glm::mat4(1.0f);

	m_target_mat = glm::translate(m_target_mat, target);

	glm::vec3 test = target - red_position_end;
	

	if (is_bone_move) {
		stop = false;
		computeAngles();
	}

}


void Bone_Animation::computeAngles()
{
	if (!stop) {


		setJacobian(0);

/*		Eigen::MatrixXf denom = Eigen::MatrixXf::Zero(9, 1);

		numer = jacobian.transpose() * goalVector;

		float n = sqrt(pow(numer(0, 0), 2) + pow(numer(1, 0), 2) + pow(numer(2, 0), 2) + pow(numer(3, 0), 2) +
			pow(numer(4, 0), 2) + pow(numer(5, 0), 2) + pow(numer(6, 0), 2) + pow(numer(7, 0), 2) +
			pow(numer(8, 0), 2));

		denom = jacobian * jacobian.transpose() * goalVector;

		float d = sqrt(pow(denom(0, 0), 2) + pow(denom(1, 0), 2) + pow(denom(2, 0), 2));

		alpha = float(n / d);
*/



		anglesVector = get_transpose(0) * goalVector;
		anglesVector = anglesVector * alpha;
	//	anglesVector = get_transpose(0) * goalVector;
		//anglesVectorZ = get_transpose(0) * (goalVector); 
		//anglesVectorX = get_transpose(2) * (goalVector);
		//anglesVectorY = get_transpose(1) * (goalVector); 

		angles[0] += anglesVector(0, 0);

		angles[1] += anglesVector(1, 0);
		angles[2] += anglesVector(2, 0);


		angles[3] += anglesVector(3, 0);
		angles[4] += anglesVector(4, 0);
		angles[5] += anglesVector(5, 0);

		angles[6] += anglesVector(6, 0);
		angles[7] += anglesVector(7, 0);
		angles[8] += anglesVector(8, 0);

		//angles[0] = fmodf(angles[0], 360);
		//angles[1] = fmodf(angles[1], 360);
		//angles[2] = fmodf(angles[2], 360);

		//angles[3] = fmodf(angles[3], 360);
		//angles[4] = fmodf(angles[4], 360);
		//angles[5] = fmodf(angles[5], 360);



		//angles[6] = fmodf(angles[6], 360);
		//angles[7] = fmodf(angles[7], 360);
		//angles[8] = fmodf(angles[8], 360);


		///std::cout << "FINAL:" << std::endl;

		//std::cout << blue_position_end[0] << std::endl;*/


		forward_kinematics(angles[6], angles[3], angles[0], angles[7], angles[4], angles[1]
			, angles[8], angles[5], angles[2]);



		shouldStop();

	}

}

void Bone_Animation::shouldStop()
{


	//	std::cout << (glm::length(target - blue_position_end))<<" "<< prevDist<< std::endl;
	if ((glm::length(target - blue_position_end) < errorFactor)) {
		stop = true;
		is_bone_move = false;
		prevDist = 100;
	}
	else {
		prevDist = glm::length(target - blue_position_end);

	}


}


void Bone_Animation::setJacobian(int id) {
	goal = target - blue_position_end;
	goalVector << goal.x, 
				  goal.y,
				  goal.z; 

	v1 = glm::cross(glm::normalize(glm::vec3(m_yellow_mat[2])), (blue_position_end - red_position_end));
	v2 = glm::cross(glm::normalize(glm::vec3(m_pink_mat[2])), (blue_position_end - yellow_position_end));
	v3 = glm::cross(glm::normalize(glm::vec3(m_blue_mat[2])), (blue_position_end - pink_position_end));


	v4 = glm::cross(glm::normalize(glm::vec3(m_yellow_mat[1])), (blue_position_end - red_position_end));
	v5 = glm::cross(glm::normalize(glm::vec3(m_pink_mat[1])), (blue_position_end - yellow_position_end));
	v6 = glm::cross(glm::normalize(glm::vec3(m_blue_mat[1])), (blue_position_end - pink_position_end));


	v7 = cross(glm::normalize(glm::vec3(m_yellow_mat[0])), (blue_position_end - red_position_end));
	v8 = cross(glm::normalize(glm::vec3(m_pink_mat[0])), (blue_position_end - yellow_position_end));
	v9 = cross(glm::normalize(glm::vec3(m_blue_mat[0])), (blue_position_end - pink_position_end));

	jacobian <<
		v1.x, v2.x, v3.x, v4.x, v5.x, v6.x, v7.x, v8.x, v9.x,
		v1.y, v2.y, v3.y, v4.y, v5.y, v6.y, v7.y, v8.y, v9.y,
		v1.z, v2.z, v3.z, v4.z, v5.z, v6.z, v7.z, v8.z, v9.z;
	
	//jacobian_matrix[2] = { { v1.x, v2.x, v3.x },
	//{ v1.y, v2.y, v3.y } ,
	//{ v1.z, v2.z, v3.z } ,
	//};

}

Eigen::MatrixXf Bone_Animation::get_transpose(int id)
{
	/*glm::vec3 num = glm::transpose(jacobian_matrix[id]) * goalVector;

	float t = sqrt(pow(num.x, 2) + pow(num.y, 2) + pow(num.z, 2));

	glm::vec3 den = jacobian_matrix[id] * glm::transpose(jacobian_matrix[id]) * goalVector;


	float p = sqrt(pow(den.x, 2) + pow(den.y, 2) + pow(den.z, 2));

	alpha = t / p;*/
	/*Eigen::MatrixXf test = Eigen::MatrixXf::Zero(9, 1);
	test = jacobian.transpose()*goalVector;*/

//	Eigen::MatrixXf aux = Eigen::MatrixXf::Ones(3,9) * alpha;

	return (jacobian.transpose());//.Transpose(); // J
													 //return m_yellow_mat;

													 //return m_yellow_mat;
}


//float Bone_Animation::distance(float x1, float y1,
//	float z1, float x2,
//	float y2, float z2)
//{
//	float d = sqrt(pow(x2 - x1, 2) +
//		pow(y2 - y1, 2) +
//		pow(z2 - z1, 2) * 1.0);
//	return d;
//}


void Bone_Animation::forward_kinematics(float angleX, float angleY, float angleZ, float anglePurpleX, float anglePurpleY,
	float anglePurpleZ, float angleBlueX, float angleBlueY,
	float angleBlueZ)
{

	glm::vec3 scale = glm::vec3(1.f, 1.f, 1.f);
	m_yellow_mat = glm::mat4(1.0f);


	m_yellow_mat = glm::rotate(m_yellow_mat, glm::radians(angleZ), glm::vec3(0, 0, 1));
	rot[0] = (glm::normalize(m_yellow_mat * glm::vec4(0, 0, 1, 0)));

	std::cout << typeid(glm::normalize(m_yellow_mat * glm::vec4(1, 0, 0, 0))).name() << std::endl;
	m_yellow_mat = glm::rotate(m_yellow_mat, glm::radians(angleY), glm::vec3(0, 1, 0));
	rot[1] = (glm::normalize(m_yellow_mat * glm::vec4(0, 1, 0, 0)));

	m_yellow_mat = glm::rotate(m_yellow_mat, glm::radians(angleX), glm::vec3(1, 0, 0));

	rot[2] = (glm::normalize(m_yellow_mat * glm::vec4(1, 0, 0, 0)));


	m_yellow_mat = glm::rotate(m_yellow_mat, glm::radians(-angleZ), glm::vec3(0, 0, 1));
	m_yellow_mat = glm::rotate(m_yellow_mat, glm::radians(-angleY), glm::vec3(0, 1, 0));

	m_yellow_mat = glm::rotate(m_yellow_mat, glm::radians(-angleX), glm::vec3(1, 0, 0));


	m_yellow_mat = glm::mat4(1.0f);


	m_yellow_mat = glm::scale(m_yellow_mat, scale);

	glm::vec3 pivot = glm::vec3(0.0f, 2.f, 0.0f);

	glm::vec3 pos = root_position;

	m_yellow_mat = glm::translate(m_yellow_mat, pos);
	m_yellow_mat = glm::rotate(m_yellow_mat, glm::radians(angleZ), glm::vec3(0, 0, 1));
	m_yellow_mat = glm::rotate(m_yellow_mat, glm::radians(angleY), glm::vec3(0, 1, 0));

	m_yellow_mat = glm::rotate(m_yellow_mat, glm::radians(angleX), glm::vec3(1, 0, 0));


	m_yellow_mat = glm::translate(m_yellow_mat, pivot);

	//-------------------------------------------------------------------------------------------------


	m_yellow_mat = glm::translate(m_yellow_mat, glm::vec3(0.0f, 2.0f, 0.0f));
	//std::cout << " yellow: " << std::endl;
	//yellow_position_end = glm::vec3(m_yellow_mat[3]);

	//std::cout << yellow_position_end.x << std::endl;
	//std::cout << yellow_position_end.y << std::endl;
	//std::cout << yellow_position_end.z << std::endl;
	//std::cout << " " << std::endl;

	m_yellow_mat = glm::translate(m_yellow_mat, glm::vec3(0.0f, -2.0f, 0.0f));


	//-------------------------------------------------------------------------------------------------


	m_yellow_mat = glm::scale(m_yellow_mat, scale_vector[1]);












	m_pink_mat = glm::mat4(1.0f);

	m_pink_mat = glm::rotate(m_pink_mat, glm::radians(angleZ), glm::vec3(0, 0, 1));




	m_pink_mat = glm::rotate(m_pink_mat, glm::radians(angleY), glm::vec3(0, 1, 0));



	m_pink_mat = glm::rotate(m_pink_mat, glm::radians(angleX), glm::vec3(1, 0, 0));

	m_pink_mat = glm::rotate(m_pink_mat, glm::radians(anglePurpleZ), glm::vec3(0, 0, 1));
	rot[4] = (glm::normalize(m_pink_mat * glm::vec4(0, 0, 1, 0)));

	m_pink_mat = glm::rotate(m_pink_mat, glm::radians(anglePurpleY), glm::vec3(0, 1, 0));

	rot[5] = (glm::normalize(m_pink_mat * glm::vec4(0, 1, 0, 0)));



	m_pink_mat = glm::rotate(m_pink_mat, glm::radians(anglePurpleX), glm::vec3(1, 0, 0));
	rot[6] = (glm::normalize(m_pink_mat * glm::vec4(1, 0, 0, 0)));



	m_pink_mat = glm::rotate(m_pink_mat, glm::radians(-angleZ), glm::vec3(0, 0, 1));
	m_pink_mat = glm::rotate(m_pink_mat, glm::radians(-angleY), glm::vec3(0, 1, 0));

	m_pink_mat = glm::rotate(m_pink_mat, glm::radians(-angleX), glm::vec3(1, 0, 0));

	m_pink_mat = glm::rotate(m_pink_mat, glm::radians(-anglePurpleZ), glm::vec3(0, 0, 1));
	m_pink_mat = glm::rotate(m_pink_mat, glm::radians(-anglePurpleY), glm::vec3(0, 1, 0));


	m_pink_mat = glm::rotate(m_pink_mat, glm::radians(-anglePurpleX), glm::vec3(1, 0, 0));



	m_pink_mat = glm::mat4(1.0f);
	m_pink_mat = glm::scale(m_pink_mat, scale);

	glm::vec3 pivot2 = glm::vec3(0.0f, 5.5f, 0.0f);

	glm::vec3 pos2 = root_position;

	m_pink_mat = glm::translate(m_pink_mat, pos2);
	m_pink_mat = glm::rotate(m_pink_mat, glm::radians(angleZ), glm::vec3(0, 0, 1));
	m_pink_mat = glm::rotate(m_pink_mat, glm::radians(angleY), glm::vec3(0, 1, 0));

	m_pink_mat = glm::rotate(m_pink_mat, glm::radians(angleX), glm::vec3(1, 0, 0));



	m_pink_mat = glm::translate(m_pink_mat, pivot2);

	glm::vec3 pp = glm::vec3(0.0f, -1.5f, 0.0f);

	m_pink_mat = glm::translate(m_pink_mat, pp);
	m_pink_mat = glm::rotate(m_pink_mat, glm::radians(anglePurpleZ), glm::vec3(0, 0, 1));
	m_pink_mat = glm::rotate(m_pink_mat, glm::radians(anglePurpleY), glm::vec3(0, 1, 0));


	m_pink_mat = glm::rotate(m_pink_mat, glm::radians(anglePurpleX), glm::vec3(1, 0, 0));


	m_pink_mat = glm::translate(m_pink_mat, -pp);


	//-------------------------------------------------------------------------------------------------


	m_pink_mat = glm::translate(m_pink_mat, glm::vec3(0.0f, 1.5f, 0.0f));
	pink_position_end = glm::vec3(m_pink_mat[3]);

	//std::cout << " pink: " << std::endl;
	//std::cout << pink_position_end.x << std::endl;
	//std::cout << pink_position_end.y << std::endl;
	//std::cout << pink_position_end.z << std::endl;
	//std::cout << " " << std::endl;

	m_pink_mat = glm::translate(m_pink_mat, glm::vec3(0.0f, -1.5f, 0.0f));


	//-------------------------------------------------------------------------------------------------


	m_pink_mat = glm::scale(m_pink_mat, scale_vector[2]);








	m_blue_mat = glm::mat4(1.0f);



	m_blue_mat = glm::rotate(m_blue_mat, glm::radians(angleZ), glm::vec3(0, 0, 1));
	m_blue_mat = glm::rotate(m_blue_mat, glm::radians(angleY), glm::vec3(0, 1, 0));

	m_blue_mat = glm::rotate(m_blue_mat, glm::radians(angleX), glm::vec3(1, 0, 0));

	m_blue_mat = glm::rotate(m_blue_mat, glm::radians(anglePurpleZ), glm::vec3(0, 0, 1));
	m_blue_mat = glm::rotate(m_blue_mat, glm::radians(anglePurpleY), glm::vec3(0, 1, 0));

	m_blue_mat = glm::rotate(m_blue_mat, glm::radians(anglePurpleX), glm::vec3(1, 0, 0));

	m_blue_mat = glm::rotate(m_blue_mat, glm::radians(angleBlueZ), glm::vec3(0, 0, 1));
	rot[7]= (glm::normalize(m_pink_mat * glm::vec4(0, 0, 1, 0)));

	m_blue_mat = glm::rotate(m_blue_mat, glm::radians(angleBlueY), glm::vec3(0, 1, 0));

	rot[8] = (glm::normalize(m_pink_mat * glm::vec4(0, 1, 0, 0)));


	m_blue_mat = glm::rotate(m_blue_mat, glm::radians(angleBlueX), glm::vec3(1, 0, 0));
	rot[9] = (glm::normalize(m_pink_mat * glm::vec4(1, 0, 0, 0)));




	m_blue_mat = glm::rotate(m_blue_mat, glm::radians(-angleZ), glm::vec3(0, 0, 1));
	m_blue_mat = glm::rotate(m_blue_mat, glm::radians(-angleY), glm::vec3(0, 1, 0));

	m_blue_mat = glm::rotate(m_blue_mat, glm::radians(-angleX), glm::vec3(1, 0, 0));

	m_blue_mat = glm::rotate(m_blue_mat, glm::radians(-anglePurpleZ), glm::vec3(0, 0, 1));
	m_blue_mat = glm::rotate(m_blue_mat, glm::radians(-anglePurpleY), glm::vec3(0, 1, 0));

	m_blue_mat = glm::rotate(m_blue_mat, glm::radians(-anglePurpleX), glm::vec3(1, 0, 0));

	m_blue_mat = glm::rotate(m_blue_mat, glm::radians(-angleBlueZ), glm::vec3(0, 0, 1));
	m_blue_mat = glm::rotate(m_blue_mat, glm::radians(-angleBlueY), glm::vec3(0, 1, 0));

	m_blue_mat = glm::rotate(m_blue_mat, glm::radians(-angleBlueX), glm::vec3(1, 0, 0));






	m_blue_mat = glm::mat4(1.0f);
	m_blue_mat = glm::scale(m_blue_mat, scale);

	glm::vec3 pivot3 = glm::vec3(0.0f, 8.f, 0.0f);

	glm::vec3 pos3 = root_position;

	m_blue_mat = glm::translate(m_blue_mat, pos3);
	m_blue_mat = glm::rotate(m_blue_mat, glm::radians(angleZ), glm::vec3(0, 0, 1));
	m_blue_mat = glm::rotate(m_blue_mat, glm::radians(angleY), glm::vec3(0, 1, 0));

	m_blue_mat = glm::rotate(m_blue_mat, glm::radians(angleX), glm::vec3(1, 0, 0));



	m_blue_mat = glm::translate(m_blue_mat, pivot3);

	glm::vec3 pp2 = glm::vec3(0.0f, -4.f, 0.0f);

	m_blue_mat = glm::translate(m_blue_mat, pp2);

	m_blue_mat = glm::rotate(m_blue_mat, glm::radians(anglePurpleZ), glm::vec3(0, 0, 1));
	m_blue_mat = glm::rotate(m_blue_mat, glm::radians(anglePurpleY), glm::vec3(0, 1, 0));

	m_blue_mat = glm::rotate(m_blue_mat, glm::radians(anglePurpleX), glm::vec3(1, 0, 0));


	m_blue_mat = glm::translate(m_blue_mat, -pp2);

	glm::vec3 pp3 = glm::vec3(0.0f, -1.f, 0.0f);

	m_blue_mat = glm::translate(m_blue_mat, pp3);

	m_blue_mat = glm::rotate(m_blue_mat, glm::radians(angleBlueZ), glm::vec3(0, 0, 1));
	m_blue_mat = glm::rotate(m_blue_mat, glm::radians(angleBlueY), glm::vec3(0, 1, 0));

	m_blue_mat = glm::rotate(m_blue_mat, glm::radians(angleBlueX), glm::vec3(1, 0, 0));

	m_blue_mat = glm::translate(m_blue_mat, -pp3);


	//-------------------------------------------------------------------------------------------------


	m_blue_mat = glm::translate(m_blue_mat, glm::vec3(0.0f, 1.f, 0.0f));
	blue_position_end = glm::vec3(m_blue_mat[3]);
	/*
	std::cout << " blue: " << std::endl;
	std::cout << blue_position_end.x << std::endl;
	std::cout << blue_position_end.y << std::endl;
	std::cout << blue_position_end.z << std::endl;
	std::cout << " " << std::endl;
	*/



	m_blue_mat = glm::translate(m_blue_mat, glm::vec3(0.0f, -1.f, 0.0f));


	//-------------------------------------------------------------------------------------------------


	m_blue_mat = glm::scale(m_blue_mat, scale_vector[3]);


}


void Bone_Animation::RotatePurple(float angleX, float angleY, float angleZ) {

	glm::vec3 scale = glm::vec3(1.f, 1.f, 1.f);
	m_pink_mat = glm::mat4(1.0f);
	m_pink_mat = glm::scale(m_pink_mat, scale);

	glm::vec3 pivot = glm::vec3(0.0f, 30.f, 0.0f);

	glm::vec3 pos = yellow_position;

	m_pink_mat = glm::translate(m_pink_mat, pos);
	m_pink_mat = glm::rotate(m_pink_mat, glm::radians(angleX), glm::vec3(1, 0, 0));
	m_pink_mat = glm::rotate(m_pink_mat, glm::radians(angleY), glm::vec3(0, 1, 0));
	m_pink_mat = glm::rotate(m_pink_mat, glm::radians(angleZ), glm::vec3(0, 0, 1));


	m_pink_mat = glm::translate(m_pink_mat, pivot);

	m_pink_mat = glm::scale(m_pink_mat, scale_vector[2]);
}






void Bone_Animation::reset()
{




	yellowXangle = 0;
	yellowYangle = 0;
	yellowZangle = 0;

	m_yellow_mat = glm::mat4(1.0f);
	m_yellow_mat = glm::translate(m_yellow_mat, yellow_position);
	m_yellow_mat = glm::scale(m_yellow_mat, scale_vector[1]);

	m_pink_mat = glm::mat4(1.0f);
	m_pink_mat = glm::translate(m_pink_mat, pink_position);
	m_pink_mat = glm::scale(m_pink_mat, scale_vector[2]);

	m_blue_mat = glm::mat4(1.0f);
	m_blue_mat = glm::translate(m_blue_mat, blue_position);
	m_blue_mat = glm::scale(m_blue_mat, scale_vector[3]);





	stop = false;
	positionChanged = false;
	time = 0.0f;
	errorFactor = 0.1f;
	//-1.43185
	//	8.11571
	//	2

	target = glm::vec3(3.f, 8.f, 3.0f);
	red_position_end = root_position;
	alpha = 0.1f;

	angles[0] = 0;
	angles[1] = 0;
	angles[2] = 0;



	angles[3] = 0;
	angles[4] = 0;
	angles[5] = 0;

	m_target_mat = glm::mat4(1.0f);
	m_target_mat = glm::translate(m_target_mat, target);


}