#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <Inventor/SoDB.h>
#include <Inventor/SoOutput.h>
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/VRMLnodes/SoVRMLAppearance.h>
#include <Inventor/VRMLnodes/SoVRMLColor.h>
#include <Inventor/VRMLnodes/SoVRMLCoordinate.h>
#include <Inventor/VRMLnodes/SoVRMLIndexedFaceSet.h>
#include <Inventor/VRMLnodes/SoVRMLMaterial.h>
#include <Inventor/VRMLnodes/SoVRMLShape.h>
#include <Inventor/VRMLnodes/SoVRMLTransform.h>

int
main(int argc, char** argv)
{
	if (argc != 3)
	{
		std::cerr << "Usage: tris2wrl [input.tris] [output.wrl]" << std::endl;
		return EXIT_FAILURE;
	}
	
	SoDB::init();
	
	std::fstream file;
	file.open(argv[1]);
	
	std::string line;
	std::vector<std::string> tokens;
	
	std::getline(file, line);
	
	if (!boost::starts_with(line, "#"))
	{
		file.close();
		return EXIT_FAILURE;
	}
	
	SoVRMLTransform* root = new SoVRMLTransform();
	root->ref();
	
	SoVRMLShape* shape = new SoVRMLShape();
	root->addChild(shape);
	
	SoVRMLAppearance* appearance = new SoVRMLAppearance();
	shape->appearance = appearance;
	
	SoVRMLMaterial* material = new SoVRMLMaterial();
	appearance->material = material;
	
	SoVRMLIndexedFaceSet* indexedFaceSet = new SoVRMLIndexedFaceSet();
	shape->geometry = indexedFaceSet;
	
	SoVRMLCoordinate* coordinate = new SoVRMLCoordinate();
	indexedFaceSet->coord = coordinate;
	
	SoVRMLColor* color = new SoVRMLColor();
	indexedFaceSet->color = color;
	
	indexedFaceSet->colorPerVertex.setValue(false);
	indexedFaceSet->solid.setValue(false);
	
	while (-1 != file.peek())
	{
		std::getline(file, line);
		
		if (line.empty())
		{
			continue;
		}
		else if (boost::starts_with(line, "#"))
		{
			break;
		}
		
		boost::trim(line);
		boost::split(tokens, line, boost::is_space(), boost::token_compress_on);
		
		coordinate->point.set1Value(coordinate->point.getNum(), std::stof(tokens[0]), std::stof(tokens[1]), std::stof(tokens[2]));
	}
	
	while (-1 != file.peek())
	{
		std::getline(file, line);
		
		if (line.empty())
		{
			continue;
		}
		else if (boost::starts_with(line, "#"))
		{
			break;
		}
		
		boost::trim(line);
		boost::split(tokens, line, boost::is_space(), boost::token_compress_on);
		
		for (std::size_t i = 0; i < tokens.size(); ++i)
		{
			indexedFaceSet->coordIndex.set1Value(indexedFaceSet->coordIndex.getNum(), std::stoi(tokens[i]));
		}
		
		indexedFaceSet->coordIndex.set1Value(indexedFaceSet->coordIndex.getNum(), -1);
	}
	
	while (-1 != file.peek())
	{
		std::getline(file, line);
		
		if (line.empty())
		{
			continue;
		}
		else if (boost::starts_with(line, "#"))
		{
			break;
		}
		
		boost::trim(line);
		boost::split(tokens, line, boost::is_space(), boost::token_compress_on);
		
		if (tokens.size() > 5)
		{
			color->color.set1Value(color->color.getNum(), std::stof(tokens[3]), std::stof(tokens[4]), std::stof(tokens[5]));
		}
	}
	
	while (-1 != file.peek())
	{
		std::getline(file, line);
		
		if (line.empty())
		{
			continue;
		}
		else if (boost::starts_with(line, "#"))
		{
			break;
		}
		
		boost::trim_if(line, boost::is_any_of(", \t\n\v\f\r"));
		boost::split(tokens, line, boost::is_any_of(", "), boost::token_compress_on);
		
		for (std::size_t i = 0; i < tokens.size(); ++i)
		{
			indexedFaceSet->colorIndex.set1Value(indexedFaceSet->colorIndex.getNum(), std::stoi(tokens[i]));
		}
	}
	
	std::cout << "coordinates: " << coordinate->point.getNum() << std::endl;
	std::cout << "faces: " << indexedFaceSet->coordIndex.getNum() / 4 << std::endl;
	std::cout << "materials: " << color->color.getNum() << std::endl;
	std::cout << "material indices: " << indexedFaceSet->colorIndex.getNum() << std::endl;
	
	if (0 == indexedFaceSet->colorIndex.getNum())
	{
		color->ref();
		indexedFaceSet->color.setValue(nullptr);
		color->unref();
	}
	
	file.close();
	
	SoOutput out;
	out.openFile(argv[2]);
	out.setHeaderString("#VRML V2.0 utf8");
	SoWriteAction wra(&out);
	wra.apply(root);
	out.closeFile();
	
	root->unref();
	
	return EXIT_SUCCESS;
}
