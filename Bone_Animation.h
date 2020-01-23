#pragma once

#include <string>
#include <vector>
#include <iostream>
#include <algorithm>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>	
#include <math.h>       /* pow */
#include <Eigen/Dense>
#include <list>
class Bone_Animation
{
public:
	Bone_Animation();
	~Bone_Animation();

	void init();
	void update(float delta_time);
	void forward_kinematics(float angleX, float angleY, float angleZ, float anglePurpleX, float anglePurpleY,
		float anglePurpleZ, float angleBlueX, float angleBlueY,
		float angleBlueZ);
	void RotatePurple(float angleX, float angleY, float angleZ);

	glm::tvec4<float>* rot = new glm::tvec4<float>[9];


	Eigen::MatrixXf jacobian = Eigen::MatrixXf::Zero(3, 9);
	Eigen::MatrixXf anglesVector = Eigen::MatrixXf::Zero(9, 1);
	Eigen::MatrixXf goalVector = Eigen::MatrixXf::Zero(3, 1);
	bool is_bone_move;
	float prevDist = 100;



	//float distance(float x1, float y1,
	//	float z1, float x2,
	//	float y2, float z2);


	//void forwardKinematics(float yellowXangle, float yellowYangle, float yellowZangle,
	//	float purpleXangle, float purpleYangle, float purpleZangle, float blueXangle,
	//	float blueYangle, float blueZangle);
	Eigen::MatrixXf get_transpose(int id);

	void reset();

	void shouldStop();
	void computeAngles();
	void setJacobian(int id);

	glm::mat4 get_yellow_mat() { return m_yellow_mat; };
	glm::mat4 get_pink_mat() { return m_pink_mat; };
	glm::mat4 get_blue_mat() { return m_blue_mat; };
	glm::mat4 get_target_mat() { return m_target_mat; };

public:

	// Here the head of each vector is the root bone
	std::vector<glm::vec3> scale_vector;
	std::vector<glm::vec3> rotation_degree_vector;
	std::vector<glm::vec4> colors;

	glm::vec3 root_position;
	glm::vec3 yellow_position;
	glm::vec3 pink_position;
	glm::vec3 blue_position;

	float yellowXangle;
	float yellowYangle;
	float yellowZangle = 30;


	float purpleXangle;
	float purpleYangle;
	float purpleZangle = 30;


	float blueXangle;
	float blueYangle;
	float blueZangle = 30;

	glm::vec3 yellowRot;
	glm::vec3 pinkRot;
	glm::vec3 blueRot;


	glm::vec3 lastTargetPosition, goal, v1, v2, v3, v4, v5, v6, v7,v8,v9, anglesVectorZ, anglesVectorX, anglesVectorY;

	float alphaYellow, alphaPink, alphaPurple;
	float angles[9];
	float time, errorFactor, alpha, limitS0, limitS1, limitS2;

	glm::vec3 pink_position_end, blue_position_end, yellow_position_end, target, red_position_end;

	bool stop, positionChanged;



private:
	glm::mat4 m_yellow_mat;
	glm::mat4 m_pink_mat;
	glm::mat4 m_target_mat;

	glm::mat4 m_blue_mat;
};