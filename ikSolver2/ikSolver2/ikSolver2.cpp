#include "ikSolver2.h"

#include <maya/MGlobal.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MTransformationMatrix.h>
#include <maya/MPlug.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MEulerRotation.h>
#include <maya/MDataHandle.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnGenericAttribute.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFloatVector.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MVector.h>
#include <maya/MMatrix.h>
#include <maya/MPoint.h>
#include <math.h>

MTypeId ikSolver2::typeId(0x80001);

MObject ikSolver2::inputTranslateX;
MObject ikSolver2::inputTranslateY;
MObject ikSolver2::inputTranslateZ;
MObject ikSolver2::inputTranslate;
MObject ikSolver2::outputRotateX;
MObject ikSolver2::outputRotateY;
MObject ikSolver2::outputRotateZ;
MObject ikSolver2::outputRotate;
MObject ikSolver2::driverMatrix;
MObject ikSolver2::upVectorMatrix;

void* ikSolver2::creator()
{
	return new ikSolver2();
}


MStatus ikSolver2::initialize()
{
	MFnNumericAttribute numFn;
	MFnTypedAttribute typedFn;
	MFnCompoundAttribute compound;
	MFnMatrixAttribute matrixFn;
	MFnUnitAttribute uAttr;

	driverMatrix = matrixFn.create("driverMatrix", "dvm");
	addAttribute(driverMatrix);

	upVectorMatrix = matrixFn.create("upVectorMatrix", "uvm");
	addAttribute(upVectorMatrix);

	inputTranslateX = numFn.create("inputTranslateX", "itx", MFnNumericData::kDouble,0);
	numFn.setStorable(true);
	numFn.setKeyable(true);
	numFn.setWritable(true);
	addAttribute(inputTranslateX);

	inputTranslateY = numFn.create("inputTranslateY", "ity", MFnNumericData::kDouble, 0);
	numFn.setStorable(true);
	numFn.setKeyable(true);
	numFn.setWritable(true);
	addAttribute(inputTranslateY);

	inputTranslateZ = numFn.create("inputTranslateZ", "itz", MFnNumericData::kDouble, 0);
	numFn.setStorable(true);
	numFn.setKeyable(true);
	numFn.setWritable(true);
	addAttribute(inputTranslateZ);

	inputTranslate = compound.create("InputTranslate", "intr");
	compound.addChild(inputTranslateX);
	compound.addChild(inputTranslateY);
	compound.addChild(inputTranslateZ);
	compound.setStorable(true);
	compound.setKeyable(true);
	compound.setWritable(true);
	addAttribute(inputTranslate);

	outputRotateX = uAttr.create("outputRotateX", "orx", MFnUnitAttribute::kAngle, 0.0);
	uAttr.setStorable(false);
	uAttr.setKeyable(false);
	uAttr.setWritable(false);
	addAttribute(outputRotateX);

	outputRotateY = uAttr.create("outputRotateY", "ory", MFnUnitAttribute::kAngle, 0.0);
	uAttr.setStorable(false);
	uAttr.setKeyable(false);
	uAttr.setWritable(false);
	addAttribute(outputRotateY);

	outputRotateZ = uAttr.create("outputRotateZ", "orz", MFnUnitAttribute::kAngle, 0.0);
	uAttr.setStorable(false);
	uAttr.setKeyable(false);
	uAttr.setWritable(false);
	addAttribute(outputRotateZ);

	outputRotate = compound.create("outputRotate", "or");
	compound.addChild(outputRotateX);
	compound.addChild(outputRotateY);
	compound.addChild(outputRotateZ);
	compound.setStorable(false);
	compound.setKeyable(false);
	compound.setWritable(false);
	addAttribute(outputRotate);

	attributeAffects(inputTranslate, outputRotate);
	attributeAffects(upVectorMatrix, outputRotate);
	attributeAffects(driverMatrix, outputRotate);

	return MS::kSuccess;
}

MStatus ikSolver2::compute(const MPlug& plug, MDataBlock& dataBlock)
{
	if ((plug == outputRotate) || (plug == outputRotateX)
		|| (plug == outputRotateY) || (plug == outputRotateZ))
	{
		MMatrix driverMatrixV = dataBlock.inputValue(driverMatrix).asMatrix();
		MMatrix upVectorMatrixV = dataBlock.inputValue(upVectorMatrix).asMatrix();
		MVector inputTranslateV = dataBlock.inputValue(inputTranslate).asVector();

		// get positions
		MVector driverMatrixPos(driverMatrixV[3][0],
			driverMatrixV[3][1],
			driverMatrixV[3][2]);
		MVector upVectorMatrixPos(upVectorMatrixV[3][0],
			upVectorMatrixV[3][1],
			upVectorMatrixV[3][2]);

		//compute needed vectors
		MVector upVec = upVectorMatrixPos - inputTranslateV;
		MVector aimVec = driverMatrixPos - inputTranslateV;
		upVec.normalize();
		aimVec.normalize();
		//compute perpendicular vectors
		MVector cross = aimVec^upVec;
		upVec = cross ^ aimVec;

		// Build rotation matrix
		double myMatrix[4][4] = { {aimVec.x, aimVec.y, aimVec.z, 0},
		{upVec.x, upVec.y, upVec.z, 0},
		{cross.x, cross.y, cross.z, 0},
		{inputTranslateV[0], inputTranslateV[1], inputTranslateV[2], 1 }
		};

		//extrac euler rotations
		MMatrix rotMatrix(myMatrix);
		MTransformationMatrix matrixFn(rotMatrix);
		MEulerRotation euler = matrixFn.eulerRotation();

		//set output data
		dataBlock.outputValue(outputRotate).set(euler.x,
			euler.y,
			euler.z);
		dataBlock.outputValue(outputRotate).setClean();

	}
	return MS::kSuccess;
}