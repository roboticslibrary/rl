#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <Inventor/SoDB.h>
#include <Inventor/SoOutput.h>
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/VRMLnodes/SoVRMLInline.h>
#include <Inventor/VRMLnodes/SoVRMLTransform.h>

int
main(int argc, char** argv)
{
	if (argc != 3)
	{
		std::cerr << "Usage: csv2wrl [input.csv] [output.wrl]" << std::endl;
		return EXIT_FAILURE;
	}
	
	SoDB::init();
	
	SoVRMLTransform* root = new SoVRMLTransform();
	root->ref();
	
	std::fstream input;
	input.open(argv[1]);
	
	for (std::string line; std::getline(input, line);)
	{
		std::istringstream stream(line);
		
		std::string name;
		std::getline(stream, name, ',');
		
		std::string number;
		std::getline(stream, number, ',');
		double x = std::atof(number.c_str());
		std::getline(stream, number, ',');
		double y = std::atof(number.c_str());
		std::getline(stream, number, ',');
		double z = std::atof(number.c_str());
		std::getline(stream, number, ',');
		double a = std::atof(number.c_str());
		std::getline(stream, number, ',');
		double b = std::atof(number.c_str());
		std::getline(stream, number, ',');
		double c = std::atof(number.c_str());
		
		Eigen::AngleAxis<double> rotation(
			Eigen::AngleAxis<double>(c * static_cast<double>(M_PI) / 180, Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxis<double>(b * static_cast<double>(M_PI) / 180, Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxis<double>(a * static_cast<double>(M_PI) / 180, Eigen::Vector3d::UnitX())
		);
		
		SoVRMLTransform* vrmlTransform = new SoVRMLTransform();
		vrmlTransform->setName(name.c_str());
		vrmlTransform->rotation.setValue(SbVec3f(rotation.axis().x(), rotation.axis().y(), rotation.axis().z()), rotation.angle());
		vrmlTransform->translation.setValue(x / 1000, y / 1000, z / 1000);
		SoVRMLInline* vrmlInline = new SoVRMLInline();
		vrmlInline->url.setValue(std::string(name + ".wrl").c_str());
		vrmlTransform->addChild(vrmlInline);
		root->addChild(vrmlTransform);
	}
	
	input.close();
	
	SoOutput output;
	output.openFile(argv[2]);
	output.setHeaderString("#VRML V2.0 utf8");
	SoWriteAction writeAction(&output);
	writeAction.apply(root);
	output.closeFile();
	
	root->unref();
	
	return EXIT_SUCCESS;
}
