#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <Inventor/SoDB.h>
#include <Inventor/SoOutput.h>
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/VRMLnodes/SoVRMLAppearance.h>
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
		std::cerr << "Usage: byu2wrl [input.g] [output.wrl]" << std::endl;
		return EXIT_FAILURE;
	}
	
	SoDB::init();
	
	std::fstream file;
	
	file.open(argv[1]);
	
	unsigned int numberParts;
	unsigned int numberVertices;
	unsigned int numberPolygons;
	unsigned int numberEdges;
	
	file >> numberParts;
	file >> numberVertices;
	file >> numberPolygons;
	file >> numberEdges;
	
	std::vector<unsigned int> beginningPolygonNumber(numberParts);
	std::vector<unsigned int> endingPolygonNumber(numberParts);
	
	for (unsigned int i = 0; i < numberParts; ++i)
	{
		file >> beginningPolygonNumber[i];
		file >> endingPolygonNumber[i];
	}
	
	std::vector<SbVec3f> vertices(numberVertices);
	
	for (unsigned int i = 0; i < numberVertices; ++i)
	{
		file >> vertices[i][0];
		file >> vertices[i][1];
		file >> vertices[i][2];
	}
	
	SoVRMLTransform* root = new SoVRMLTransform();
	root->ref();
	
	for (unsigned int i = 0; i < numberParts; ++i)
	{
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
		
		int polygon;
		unsigned int polygonIndex = 0;
		
		for (unsigned int j = beginningPolygonNumber[i] - 1; j < endingPolygonNumber[i]; ++j)
		{
			do
			{
				file >> polygon;
				
				indexedFaceSet->coordIndex.set1Value(polygonIndex, std::abs(polygon) - 1);
				++polygonIndex;
				
				coordinate->point.set1Value(std::abs(polygon) - 1, vertices[abs(polygon) - 1]);
			}
			while (polygon > 0);
			
			indexedFaceSet->coordIndex.set1Value(polygonIndex, -1);
			++polygonIndex;
		}
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
