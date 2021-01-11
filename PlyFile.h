#pragma once

#include "rply.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <cstdlib>
#define M_PI   3.14159265358979323846264338327950288



	struct Vertex{
		Eigen::Vector3d location;
		Eigen::Vector3i colour;
		Eigen::Vector3d normal;
		double curvature;

		void print(){
			std::cout << location << std::endl;
			std::cout << colour << std::endl;
			std::cout << normal << std::endl;
		}

		double EuclideanDistance(Vertex s){
			Eigen::Vector3d tV = this->location;
			Eigen::Vector3d sV = s.location;

			return (tV - sV).norm();
		}

		const bool operator ==(Vertex v){
			if((this->location == v.location) && (this->colour == v.colour) && (this->normal == v.normal)){
				return true;
			}
			return false;
		}
	};

class PlyFile{

private:

		std::vector<Vertex> points_;
		Eigen::Vector3d closestPoint(Vertex x);

public:

		PlyFile(std::vector<Vertex> init);
		PlyFile(std::string pathToPly);
		PlyFile();

		void swap(int i, int j);

		void sortAlongAxis(int axis, int low, int high);

		int partition(int axis, int high, int low);

		bool read(const std::string& filename);

		bool write(const std::string& filename);

		bool writeBlue(const std::string& filename);

		bool writeRed(const std::string& filename);

		std::vector<Vertex> getPoints();

		Vertex getPointAt(int i);

		Vertex getRandomPoint();

		void print();

		int size();

		void reColour(int r, int g, int b);

		PlyFile colourThreshold(Eigen::Vector3i colour, double threshold, std::string filename);

		double curvatureAtPoint(int index);

		void order();

		void clear();

		bool check(const std::string& filename);

		Eigen::Matrix3d changeOfBasis(Eigen::Matrix3d toBasis, Eigen::Matrix3d fromBasis);

		void representUnderChangeBasis(Eigen::Matrix3d toBasis, Eigen::Matrix3d fromBasis, Eigen::Vector3d centroid);

		void updateLocation(Eigen::Vector3d update, int index);

		void translateCloud(Eigen::Vector3d translation);

		void rotateCloud(Eigen::Matrix3d rotation);

		Eigen::Vector3d centroid();

		void translateToOrigin(Eigen::Vector3d centr);

		void rotateOrigin(Eigen::Matrix3d rotation);

		void orientateAroundYAxis();

		void rotateAxisAboutPoint(int axis, double amount, Eigen::Vector3d point);


		std::vector<Vertex> collectPositiveVertices(int axis);
		std::vector<Vertex> collectNegativeVertices(int axis);

		Eigen::Matrix3d covariance();

		void augment(PlyFile toAugment);

		void rotateAboutPoint(Eigen::Matrix3d rotation, Eigen::Vector3d point);

		void rotateAxis(int axis, double amount);

		const Vertex& operator[](size_t i){
			return points_[i];
		}

		 PlyFile operator+(PlyFile p){
			std::vector<Vertex> newPly;
//
			//std::cout << " IN here";
			for(int i =0 ; i < size(); i++){
			//	std::cout <<"first domain" << std::endl;
				newPly.push_back(points_[i]);
			}
		//	std::cout << "next domain" << std::endl;
			for(int i = 0; i < p.size(); i++){
				//std::cout << "second domain" << std::endl;
				newPly.push_back(p[i]);
			}

			std::cout << "init augment" << std::endl;
			PlyFile augPly(newPly);

			return augPly;
		}

		const Vertex& operator[](size_t i) const {
			return points_[i];
		}

		void push_back(Vertex vertex){
			points_.push_back(vertex);
		}


};


