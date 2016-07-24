//test commit
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

MObject ikSolver2::joint1initLength;
MObject ikSolver2::joint2initLength;
MObject ikSolver2::jnt1OutputRotateX;
MObject ikSolver2::jnt1OutputRotateY;
MObject ikSolver2::jnt1OutputRotateZ;
MObject ikSolver2::jnt1OutputRotate;
MObject ikSolver2::jnt2OutputRotateX;
MObject ikSolver2::jnt2OutputRotateY;
MObject ikSolver2::jnt2OutputRotateZ;
MObject ikSolver2::jnt2OutputRotate;
MObject ikSolver2::effectorMatrix;
MObject ikSolver2::poleVectorMatrix;
MObject ikSolver2::baseMatrix;


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

	baseMatrix = matrixFn.create("baseMatrix", "bsm");
	addAttribute(baseMatrix);

	poleVectorMatrix = matrixFn.create("poleVectorMatrix", "pvm");
	addAttribute(poleVectorMatrix);

	effectorMatrix = matrixFn.create("effectorMatrix", "efm");
	addAttribute(effectorMatrix);

	jnt1OutputRotateX = uAttr.create("jnt1OutputRotateX", "jnt1rx", MFnUnitAttribute::kAngle, 0.0);
	uAttr.setStorable(false);
	uAttr.setKeyable(false);
	uAttr.setWritable(false);
	addAttribute(jnt1OutputRotateX);

	jnt1OutputRotateY = uAttr.create("jnt1OutputRotateY", "jnt1ry", MFnUnitAttribute::kAngle, 0.0);
	uAttr.setStorable(false);
	uAttr.setKeyable(false);
	uAttr.setWritable(false);
	addAttribute(jnt1OutputRotateY);

	jnt1OutputRotateZ = uAttr.create("jnt1OutputRotateZ", "jnt1rz", MFnUnitAttribute::kAngle, 0.0);
	uAttr.setStorable(false);
	uAttr.setKeyable(false);
	uAttr.setWritable(false);
	addAttribute(jnt1OutputRotateZ);

	jnt1OutputRotate = compound.create("jnt1OutputRotate", "jnt1or");
	compound.addChild(jnt1OutputRotateX);
	compound.addChild(jnt1OutputRotateY);
	compound.addChild(jnt1OutputRotateZ);
	compound.setStorable(false);
	compound.setKeyable(false);
	compound.setWritable(false);
	addAttribute(jnt1OutputRotate);

	jnt2OutputRotateX = uAttr.create("jnt2OutputRotateX", "jnt2rx", MFnUnitAttribute::kAngle, 0.0);
	uAttr.setStorable(false);
	uAttr.setKeyable(false);
	uAttr.setWritable(false);
	addAttribute(jnt2OutputRotateX);

	jnt2OutputRotateY = uAttr.create("jnt2OutputRotateY", "jnt2ry", MFnUnitAttribute::kAngle, 0.0);
	uAttr.setStorable(false);
	uAttr.setKeyable(false);
	uAttr.setWritable(false);
	addAttribute(jnt2OutputRotateY);

	jnt2OutputRotateZ = uAttr.create("jnt2OutputRotateZ", "jnt2rz", MFnUnitAttribute::kAngle, 0.0);
	uAttr.setStorable(false);
	uAttr.setKeyable(false);
	uAttr.setWritable(false);
	addAttribute(jnt2OutputRotateZ);

	jnt2OutputRotate = compound.create("jnt2OutputRotate", "jnt2or");
	compound.addChild(jnt2OutputRotateX);
	compound.addChild(jnt2OutputRotateY);
	compound.addChild(jnt2OutputRotateZ);
	compound.setStorable(false);
	compound.setKeyable(false);
	compound.setWritable(false);
	addAttribute(jnt2OutputRotate);

	attributeAffects(baseMatrix, jnt2OutputRotate);
	attributeAffects(poleVectorMatrix, jnt2OutputRotate);
	attributeAffects(effectorMatrix, jnt2OutputRotate);

	attributeAffects(baseMatrix, jnt1OutputRotate);
	attributeAffects(poleVectorMatrix, jnt1OutputRotate);
	attributeAffects(effectorMatrix, jnt1OutputRotate);

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