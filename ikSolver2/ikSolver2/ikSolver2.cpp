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

	joint1initLength = numFn.create("joint1initLength", "jnt1l", MFnNumericData::kDouble, 0);
	numFn.setStorable(true);
	numFn.setKeyable(true);
	numFn.setMin(0.001);
	addAttribute(joint1initLength);

	joint2initLength = numFn.create("joint2initLength", "jnt2l", MFnNumericData::kDouble, 0);
	numFn.setStorable(true);
	numFn.setKeyable(true);
	numFn.setMin(0.001);
	addAttribute(joint2initLength);

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
	if ((plug == jnt2OutputRotate) || (plug == jnt1OutputRotate))
	{
		MMatrix baseMatrixV = dataBlock.inputValue(baseMatrix).asMatrix();
		MMatrix poleVectorMatrixV = dataBlock.inputValue(poleVectorMatrix).asMatrix();
		MMatrix effectorMatrixV = dataBlock.inputValue(effectorMatrix).asMatrix();

		//get initial length input
		double joint1initLengthV = dataBlock.inputValue(joint1initLength).asDouble();
		double joint2initLengthV = dataBlock.inputValue(joint2initLength).asDouble();

		//compute total chain length

		//MVector inputTranslateV = dataBlock.inputValue(inputTranslate).asVector();

		// get positions
		MVector baseMatrixPos(baseMatrixV[3][0],
			baseMatrixV[3][1],
			baseMatrixV[3][2]);
		MVector poleVectorMatrixPos(poleVectorMatrixV[3][0],
			poleVectorMatrixV[3][1],
			poleVectorMatrixV[3][2]);

		MVector effectorMatrixPos(effectorMatrixV[3][0],
			effectorMatrixV[3][1],
			effectorMatrixV[3][2]);

		double chainLenght = joint1initLengthV + joint2initLengthV;


		//compute needed vectors
		MVector baseEff = effectorMatrixPos -baseMatrixPos ;
		MVector basePoleVec = poleVectorMatrixPos - baseMatrixPos;
		
		//calculate rotation plane , rotapion plane vector x is baseEff
		MVector rotPlaneY = baseEff ^ basePoleVec;
		MVector rotPlaneZ = rotPlaneY ^ baseEff;

		if (chainLenght > baseEff.length())
		{
			// calculate ik here
			// b = joint1 lenght
			// a = joint2 lenght
			// c = distance from base to eff
			// cos A = (-a^2 + b^2 +c^2)/(2bc)
			// this will give me the angle to rotate the baseEff vector around previously 
			// calculated y axis vector of the rotation plane

			// joint 2 will only need to aim at the effector
			// joint 3 doesnt need to do anything
		}
		

		// this part is left overs from the aim node just so i remember how to 
		// set rotations, and it can be ignored for now

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