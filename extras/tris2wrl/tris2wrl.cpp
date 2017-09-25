#include <iostream>
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
		std::cerr << "Usage: tris2wrl [input.tris] [output.wrl]" << std::endl;
		return EXIT_FAILURE;
	}
	
	FILE* f = fopen(argv[1], "r");
	
	SoDB::init();
	
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
	
	char line[256];
	char ign;
	
	fscanf(f, "%[^\n]%c", line, &ign);
	
	float pt[3];
	unsigned int points = 0;
	
	while (EOF != fscanf(f, "%[^\n]%c", line, &ign))
	{  
		if ('#' != line[0])
		{
			// Not a comment
			if (3 != sscanf(line, "%g %g %g", pt, pt + 1, pt + 2))
			{
				fprintf(stderr, "Error couldn't parse a point (%s)\n", line);
			}
			else
			{
				coordinate->point.set1Value(points, pt[0], pt[1], pt[2]);
				++points;
			}
		}
		else
		{
			break;
		}
	}
	
	int tri[3];
	unsigned int triangles = 0;
	
	while (EOF != fscanf(f, "%[^\n]%c", line, &ign)) 
	{
		if ('#' != line[0])
		{
			// Not a comment
			if (3 != sscanf(line, "%d %d %d", tri, tri + 1, tri + 2))
			{
				fprintf(stderr, "Error couldn't parse a triangle (%s)\n", line);
			}
			else
			{
				indexedFaceSet->coordIndex.set1Value(triangles, tri[0]);
				++triangles;
				indexedFaceSet->coordIndex.set1Value(triangles, tri[1]);
				++triangles;
				indexedFaceSet->coordIndex.set1Value(triangles, tri[2]);
				++triangles;
				indexedFaceSet->coordIndex.set1Value(triangles, -1);
				++triangles;
			}
		}
		else
		{
			break;
		}
	}
	
	float nm[14];
	
	while (EOF != fscanf(f, "%[^\n]%c", line, &ign))
	{  
		if ('#' != line[0])
		{
			// Not a comment
			if (14 != sscanf(line, "%f %f %f %g %g %g %g %g %g %g %g %g %g %g", &nm[0], &nm[1], &nm[2], &nm[3], &nm[4], &nm[5], &nm[6], &nm[7], &nm[8], &nm[9], &nm[10], &nm[11], &nm[12], &nm[13])) 
			{
				fprintf(stderr, "Error couldn't parse a material (%s)\n", line);
			}
			else 
			{
			}
		}
		else
		{
			break;
		}
	}
	
	int index;
	int i = 0;
	char* pointer;
	
	while (EOF != fscanf(f, "%[^\n]%c", line, &ign)) 
	{
		if ('#' != line[0])
		{
			// Not a comment
			pointer = &line[0];
			
			while (1 == sscanf(pointer, "%d, ", &index))
			{
				i++;
				pointer += 3;
			}
		}
		else
		{
			break;
		}
	}
	
	SoOutput out;
	out.openFile(argv[2]);
	out.setHeaderString("#VRML V2.0 utf8");
	SoWriteAction wra(&out);
	wra.apply(root);
	out.closeFile();
	
	root->unref();
	
	fclose(f);
	
	return EXIT_SUCCESS;	
}
