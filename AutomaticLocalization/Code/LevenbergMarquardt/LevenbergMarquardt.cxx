/***** Levenberg Marquardt �@�̃I�t���C���� *****/
// ���f���_�Q�ƒT���Ώۓ_�Q��2�ƁA�S�T���ɂ�蓾��ꂽR,t�̃f�[�^��ǂݍ��݁A�œK���ɂ��ڍׂȈʒu���킹���s��

// �y �R�}���h���C������ �z
// 1. �T���Ώۓ_�Q�t�@�C����.pcd
// 2. ���f���_�Q�t�@�C����.pcd
// 3. �S�T�����ʂ�R,t�̃t�@�C����.csv

// ��1�F
// Input/PointData/LiverID_0002V2U0.01L0.01R20.pcd Input/PointData/R6of72LiverID_0002V2U0.01L0.01R20.pcd Output/ExhaustiveSearchResult/SearchingParameters.csv


/*** �C���N���[�h ***/

/* C */
#include <math.h>
#include <stdio.h>
#include <math.h>
#include <float.h>
#include <limits.h>

/* C++ */
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <utility>

/* PCL 1.6.0 */
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>			// KdTree
#include <pcl/kdtree/kdtree_flann.h>	// KdTree

/* �ݒ�w�b�_�t�@�C�� */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** ���O��Ԃ̐錾 ***/

/* C++ */
using namespace std;


/*** ���C�������֐� ***/
int main( int argc, char **argv ){


	/* �ϐ��̐錾 */
	FILE *inputFile;						// ���̓t�@�C��
	const char *inputFileName1 = argv[ 1 ];	// �T���Ώۓ_�Q�t�@�C����
	const char *inputFileName2 = argv[ 2 ];	// ���f���_�Q�t�@�C����
	const char *inputFileName3 = argv[ 3 ];	// �S�T������ R, t �̃t�@�C����
	Eigen::Matrix< double, 3, 3 > ER;		// �S�T�����ʂ̉�]�s��
	Eigen::Vector3d Et;						// �S�T�����ʂ̕��i�x�N�g��
	float inp1, inp2, inp3, inp4;			// �ꎞ�i�[�p�ϐ�
	int n = 0;								// �J�E���^�ϐ�

	/* �_�Q�����i�[����\���̕ϐ��̒�` */
	PointCloudInfo modelInformation;	// ���f���_�Q
	PointCloudInfo targetInformation;	// �T���Ώۓ_�Q



	/*** �v���O�����J�n�̃R�[�� ***/
	cout << "�y �v���O�����J�n �z" << endl;



	/*** �_�Q�̓ǂݍ��݁FPCD �f�[�^ ***/
	cout << "�y �_�Q�̃f�[�^�̓ǂݍ��� �z" << endl;
	
	/* �T���Ώۓ_�Q�f�[�^�̓ǂݍ��� */
	cout << ">>> �T���Ώۓ_�Q( PointXYZ )�̓Ǎ���..." << "\r";
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloudT( new pcl::PointCloud< pcl::PointXYZ > );	// ���͓_�Q( PointXYZ )�̃������m��
	if( pcl::io::loadPCDFile( inputFileName1, *cloudT ) == -1 ){ cout << ">>> �T���Ώۓ_�Q�f�[�^�̓ǂݍ��ݎ��s" << endl; return 1; }
	cout << ">>> �T���Ώۓ_�Q( PointXYZ )�̓ǂݍ��� OK" << endl;

	/* ���f���_�Q�f�[�^�̓ǂݍ��� */
	cout << ">>> ���f���_�Q( PointXYZ )�̓Ǎ���..." << "\r";
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloudM( new pcl::PointCloud< pcl::PointXYZ > );	// ���͓_�Q( PointXYZ )�̃������m��
	if( pcl::io::loadPCDFile( inputFileName2, *cloudM ) == -1 ){ cout << ">>> ���f���_�Q�f�[�^�̓ǂݍ��ݎ��s" << endl;return 1; }
	cout << ">>> ���f���_�Q( PointXYZ )�̓ǂݍ��� OK" << endl;



	/*** �S�T�����ʂ̓ǂݍ��� ***/
	cout << "�y �S�T�����ʂ� R, t �̓ǂݍ��� �z" << endl;
	cout << ">>> �S�T�����ʂ̓Ǎ���..." << "\r";
	
	/* �t�@�C���ǂݍ��ݎ��s�� */
	if( ( inputFile = fopen( inputFileName3, "r" ) ) == NULL ){
		cout << "�S�T�����ʃf�[�^�̓ǂݍ��ݎ��s" << endl;
		exit( 0 );
	}

	/* �t�@�C�����e�̓ǂݍ��� */
	while( 1 ){
		if( n == 0 ) fscanf( inputFile, "%c,,,%c,%c,%c", &inp1, &inp2, &inp3, &inp4 );				// 1�s�ڕ����͓ǂݔ�΂�
		else if( n > 0 ){
			fscanf( inputFile, "%f,%f,%f,%f", &inp1, &inp2, &inp3, &inp4 );							// 2�s�ڈȉ�����ǂݍ���
			ER( n - 1, 0 ) = inp1; ER( n - 1, 1 ) = inp2; ER( n - 1, 2 ) = inp3; Et( n - 1 ) = inp4;	// �t�@�C�����e�� R, t �Ɋi�[
		}
		if( n == 3 ) break;
		n++;
	}

	/* �������̉�� */
	fclose( inputFile );
	cout << ">>> �S�T�����ʂ̓ǂݍ��� OK" << endl;
	


	/*** �_�Q�̍��W�ϊ� ***/
	cout << "�y �_�Q�̍��W�ϊ� �z" << endl;
	cout << ">> �_�Q�̏d�S�����_���W�ɐݒ�" << endl;

	/* ���W�ϊ���̓_�Q�̃������m�� */
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr targetPointCloud( new pcl::PointCloud< pcl::PointXYZRGB > );		// �d�S�ϊ���̒T���Ώۓ_�Q
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr modelPointCloud( new pcl::PointCloud< pcl::PointXYZRGB > );		// �d�S�ϊ���̃��f���_�Q

	/* �_�Q�̍��W�ϊ� */
	CoordinateTransformation(
		&modelInformation,	// ���f���_�Q���
		&targetInformation,	// �T���Ώۓ_�Q���
		cloudM,				// ���W�ϊ��O�̃��f���_�Q
		cloudT,				// ���W�ϊ��O�̒T���Ώۓ_�Q
		modelPointCloud,	// ���W�ϊ���̃��f���_�Q
		targetPointCloud	// ���W�ϊ���̒T���Ώۓ_�Q
	);



	/*** �}�[�J�[�g�@�ɂ��œK�� ***/
	cout << "�y �}�[�J�[�g�@�ɂ��œK�� �z" << endl;

	/* �ϐ��̐錾 */
	double C = 0;							// ���݂̕]���l C
	double nextC = 0;						// ���̕]���l C
	double absC = DBL_MAX;					// ���݂̕]���l C �Ǝ��̕]���l C �̍��̐�Βl�F| nextC - C |
	double absA = DBL_MAX;					// ���݂̃p�����[�^�x�N�g�� A �Ǝ��̃p�����[�^�x�N�g�� A �̍��̐�Βl�F| nextA - A |
	double lambda = LAMBDA;					// �}�[�J�[�g�@�̌W����
	int loopCounter = 0;					// ���[�v��
	Eigen::VectorXd A( 6 );					// ���݂̃p�����[�^�x�N�g���FA = [ r1, r2, r3, t1, t2, t3 ]T
	Eigen::VectorXd nextA( 6 );				// ���̃p�����[�^�x�N�g���FnextA = [ r1, r2, r3, t1, t2, t3 ]T
	Eigen::VectorXd difA( 6 );				// ���݂̃p�����[�^�x�N�g�� A �Ǝ��̃p�����[�^�x�N�g�� A �̍�
	Eigen::VectorXd dfA( 6 );				// �]���֐� C �̈�K�����FdfA = [ ��C/��r1, ��C/r2, ��C/r3, ��C/��t1, ��C/��t2, ��C/��t3 ]T
	Eigen::Matrix< double, 3, 3 > initR;	// �}�[�J�[�g�@�� R �̏����ʒu
	Eigen::Vector3d initT;					// �}�[�J�[�g�@�� t �̏����ʒu
	Eigen::Matrix< double, 3, 3 > R;		// R
	Eigen::Vector3d t;						// t
	Eigen::Matrix< double, 3, 3 > nextR;	// nextR
	Eigen::Vector3d nextT;					// nextT
	Eigen::Matrix< double, 6, 6 > H;		// 6 �~ 6 �w�b�Z�s��
	Eigen::Matrix< double, 6, 6 > I;		// 6 �~ 6 �P�ʍs��
	Eigen::Vector3d Rodrigues;				// Rodrigues ��p������]�\�� r = [ r1, r2, r3 ]T
	Eigen::Matrix< double, 3, 3 > dR[ 3 ];	// r1, r2, r3 ���ꂼ��ɂ���]�s��̕Δ������ʂ� 3 �~ 3 �s��
	vector< int > index( 1 );				// �ŋߓ_�̃C���f�b�N�X
	vector< float > distance2( 1 );			// �ŋߓ_�܂ł̋����̓��
	pcl::PointXYZRGB transformedModelPoint;	// R, t �ɂ��ϊ���̃��f���_���W
	Eigen::Vector3d ap;						// R, t �ɂ��ϊ���̃��f���_���W
	Eigen::Vector3d bp;						// R, t �ɂ��ϊ��O�̃��f���_���W
	Eigen::Vector3d Yi;						// Yi
	Eigen::Vector3d tYi;					// t - Yi
	Eigen::Vector3d Xi;						// Xi
	Eigen::Vector3d RXitYi;					// R * Xi + t - Yi
	Eigen::Vector3d YiRXit;					// Yi - ( R * Xi + t )
	double norm;							// || Yi - ( R * Xi + t ) ||
	double norm2;							// || Yi - ( R * Xi + t ) || ��2��
	Eigen::VectorXd jacob( 6 );				// ���R�r�A��
	Eigen::Matrix< double, 6, 6 > jacob2;	// jacob * jacobT
	Eigen::Matrix< double, 6, 6 > lambdaI;	// ��I
	Eigen::Matrix< double, 6, 6 > lambdaIH;	// ��I + H
	Eigen::Matrix< double, 6, 6 > lambdaIHinv;	// ( ��I + H )-1
	int flag = 0;

	/* KdTree �̍쐬 */
	pcl::KdTreeFLANN< pcl::PointXYZRGB > kdtree;	// KdTree �N���X�̃I�u�W�F�N�g�ϐ��̐錾
	kdtree.setInputCloud( targetPointCloud );		// KdTree �ɑ΂��ē_�Q���Z�b�g

	/* R �� t �̏����l�̐ݒ� */
	initR = ER;	// �S�T�����ʂ� R �������l�ɐݒ�
	initT = Et;	// �S�T�����ʂ� t �������l�ɐݒ�

	/* �P�ʍs��̍쐬 */
	I = Eigen::MatrixXd::Identity( 6, 6 );	// �P�ʍs��̍쐬
	H = Eigen::MatrixXd::Zero( 6, 6 );		// �w�b�Z�s��̑S�v�f�� 0 �ɏ�����
	dfA = Eigen::VectorXd::Zero( 6 );		// �]���֐� C �̈�K�����x�N�g���̑S�v�f�� 0 �ɏ�����

	/* �����p�����[�^�x�N�g�� A �̎Z�o */

	/* Rodrigues �̉�]�x�N�g�� r = [ r1, r2, r3 ]T �̍쐬 */
	Rodrigues = ConvertRodrigues( &initR );

	/* ��]�s��̕Δ��� */
	PartialDerivativeForRodrigues( &Rodrigues, dR );	// ��]�̂R�v�f r1, r2, r3 �Ŕ����������ʂ��擾���AdR[ 3 ] �Ɋi�[

	/* R, t �̐ݒ� */
	R = ReconstructRFromRodrigues( &Rodrigues );	// Rodrigues �̉�]�x�N�g������̉�]�s��𕜌�
	t = initT;

	/* �p�����[�^�x�N�g�� A �̐ݒ� */
	A( 0 ) = Rodrigues( 0 );	// r1
	A( 1 ) = Rodrigues( 1 );	// r2
	A( 2 ) = Rodrigues( 2 );	// r3
	A( 3 ) = t( 0 );			// t1
	A( 4 ) = t( 1 );			// t2
	A( 5 ) = t( 2 );			// t3


	/* �����̕]���l C �̎Z�o */
	C = 0;
	for( int i = 0; i < modelPointCloud->points.size(); i++ ){

		/* R, t �ɂ�郂�f���_���W�̕ϊ� */
		bp( 0 ) = modelPointCloud->points[ i ].x;
		bp( 1 ) = modelPointCloud->points[ i ].y;
		bp( 2 ) = modelPointCloud->points[ i ].z;
		ap = R * bp + t;
		transformedModelPoint.x = ap( 0 ); transformedModelPoint.r = modelPointCloud->points[ i ].r;
		transformedModelPoint.y = ap( 1 ); transformedModelPoint.g = modelPointCloud->points[ i ].g;
		transformedModelPoint.z = ap( 2 ); transformedModelPoint.b = modelPointCloud->points[ i ].b;

		/* KdTree �ɂ��Ή��_�T�� */
		kdtree.nearestKSearch( transformedModelPoint, 1, index, distance2 );	// ���f���_�Q�̊e�_����T���Ώۓ_�Q���̍ŋߓ_�܂ł̋����̎Z�o

		/* �]���l C �̎Z�o */
		C += distance2[ 0 ];

		/* Yi �� Xi */
		Yi( 0 ) = targetPointCloud->points[ index[ 0 ] ].x; Xi( 0 ) = modelPointCloud->points[ i ].x;
		Yi( 1 ) = targetPointCloud->points[ index[ 0 ] ].y; Xi( 1 ) = modelPointCloud->points[ i ].y;
		Yi( 2 ) = targetPointCloud->points[ index[ 0 ] ].z; Xi( 2 ) = modelPointCloud->points[ i ].z;

		/* t - Yi */
		tYi = t - Yi;

		/* RXi + t - Yi */
		RXitYi = R * Xi + t - Yi;

		/* ���z�x�N�g�� */
		jacob( 0 ) = tYi.transpose() * dR[ 0 ] * Xi;
		jacob( 1 ) = tYi.transpose() * dR[ 1 ] * Xi;
		jacob( 2 ) = tYi.transpose() * dR[ 2 ] * Xi;
		jacob( 3 ) = RXitYi( 0 );
		jacob( 4 ) = RXitYi( 1 );
		jacob( 5 ) = RXitYi( 2 );

		/* �]���֐� C �̈�K���� */
		dfA += jacob;	// dfA = [ ��C/��r1, ��C/r2, ��C/r3, ��C/��t1, ��C/��t2, ��C/��t3 ]T
	}


	/* �����̃w�b�Z�s��̎Z�o */
	H = dfA * dfA.transpose();


	/* �����̎��̃p�����[�^�x�N�g���̎Z�o */
	lambdaI = lambda * I.array();		// ��I
	lambdaIH = lambdaI + H;				// ��I + H
	lambdaIHinv = lambdaIH.inverse();	// ( ��I + H )-1
	nextA = A - lambdaIHinv * dfA;		// ���̃p�����[�^�x�N�g�� A


	/* �����̎��̉�]�s��A���i�x�N�g���̎Z�o */
	Rodrigues( 0 ) = nextA( 0 );	// r1
	Rodrigues( 1 ) = nextA( 1 );	// r2
	Rodrigues( 2 ) = nextA( 2 );	// r3
	nextT( 0 ) = nextA( 3 );		// t1
	nextT( 1 ) = nextA( 4 );		// t2
	nextT( 2 ) = nextA( 5 );		// t3
	nextR = ReconstructRFromRodrigues( &Rodrigues );	// Rodrigues �̉�]�x�N�g������̉�]�s�� R �𕜌�


	/* �����̎��̕]���l C �̎Z�o */
	nextC = 0;
	for( int i = 0; i < modelPointCloud->points.size(); i++ ){

		/* R, t �ɂ�郂�f���_���W�̕ϊ� */
		bp( 0 ) = modelPointCloud->points[ i ].x;
		bp( 1 ) = modelPointCloud->points[ i ].y;
		bp( 2 ) = modelPointCloud->points[ i ].z;
		ap = R * bp + t;
		transformedModelPoint.x = ap( 0 ); transformedModelPoint.r = modelPointCloud->points[ i ].r;
		transformedModelPoint.y = ap( 1 ); transformedModelPoint.g = modelPointCloud->points[ i ].g;
		transformedModelPoint.z = ap( 2 ); transformedModelPoint.b = modelPointCloud->points[ i ].b;

		/* KdTree �ɂ��Ή��_�T�� */
		kdtree.nearestKSearch( transformedModelPoint, 1, index, distance2 );	// ���f���_�Q�̊e�_����T���Ώۓ_�Q���̍ŋߓ_�܂ł̋����̎Z�o

		/* Yi �� Xi */
		Yi( 0 ) = targetPointCloud->points[ index[ 0 ] ].x; Xi( 0 ) = modelPointCloud->points[ i ].x;
		Yi( 1 ) = targetPointCloud->points[ index[ 0 ] ].y; Xi( 1 ) = modelPointCloud->points[ i ].y;
		Yi( 2 ) = targetPointCloud->points[ index[ 0 ] ].z; Xi( 2 ) = modelPointCloud->points[ i ].z;

		/* ���̕]���l C �̎Z�o */
		YiRXit = Yi - ( nextR * Xi + nextT );	// Yi - ( R * Xi + t )
		norm = YiRXit.norm();					// || Yi - ( R * Xi + t ) ||
		norm2 = pow( norm, 2 );					// || Yi - ( R * Xi + t ) || ��2��
		nextC += norm2;							// ���̕]���l C
	}


	cout << "InitC: " << endl << C << endl;
	cout << "A: " << endl << A << endl;
	cout << "InitNextC: " << endl << nextC << endl;
	cout << "nextA: " << endl << nextA << endl;


	/* �}�[�J�[�g�@�ɂ��œK���̃��[�v */
	while( 1 ){


		/* ���[�v�񐔂̃R�}���h���C���o�� */
		cout << "---------- " << loopCounter + 1 << " ----------" << endl;

		
		/* ���̕]���l C �����P���ꂽ�ꍇ */
		if( nextC < C ){

			/* ������ */
			H = Eigen::MatrixXd::Zero( 6, 6 );		// �w�b�Z�s��̑S�v�f�� 0 �ɏ�����
			dfA = Eigen::VectorXd::Zero( 6 );		// �]���֐� C �̈�K�����x�N�g���̑S�v�f�� 0 �ɏ�����

			absC = fabs( nextC - C );			// ���݂̕]���l C �Ǝ��̕]���l C �̍��̐�Βl�F| nextC - C |
			difA = nextA - A;
			norm = difA.norm();
			absA = fabs( norm );

			cout << "C: " << endl << C << endl;
			cout << "A: " << endl << A << endl;
			cout << "NextC: " << endl << nextC << endl;
			cout << "NextA: " << endl << nextA << endl;

			if( absC < THRESHOLD_C ) break;		// �]���l C �̎����ɂ��œK���I��
			if( absA < THRESHOLD_A ) break;		// �p�����[�^�x�N�g�� A �̎����ɂ��œK���I��

			/* �X�V */
			C = nextC;							// ���݂̕]���l C ���X�V
			A = nextA;							// ���݂̃p�����[�^�x�N�g�� A ���X�V
			R = nextR;							// ���݂̉�]�s��̍X�V
			t = nextT;							// ���݂̕��i�x�N�g���̍X�V
			lambda = lambda * 0.1;				// �W���ɂ̒l�� 0.1 �{�F�j���[�g���@�̉e�����剻

			for( int i = 0; i < modelPointCloud->points.size(); i++ ){

				/* R, t �ɂ�郂�f���_���W�̕ϊ� */
				bp( 0 ) = modelPointCloud->points[ i ].x;
				bp( 1 ) = modelPointCloud->points[ i ].y;
				bp( 2 ) = modelPointCloud->points[ i ].z;
				ap = R * bp + t;
				transformedModelPoint.x = ap( 0 ); transformedModelPoint.r = modelPointCloud->points[ i ].r;
				transformedModelPoint.y = ap( 1 ); transformedModelPoint.g = modelPointCloud->points[ i ].g;
				transformedModelPoint.z = ap( 2 ); transformedModelPoint.b = modelPointCloud->points[ i ].b;

				/* KdTree �ɂ��Ή��_�T�� */
				kdtree.nearestKSearch( transformedModelPoint, 1, index, distance2 );	// ���f���_�Q�̊e�_����T���Ώۓ_�Q���̍ŋߓ_�܂ł̋����̎Z�o

				/* Yi �� Xi */
				Yi( 0 ) = targetPointCloud->points[ index[ 0 ] ].x; Xi( 0 ) = modelPointCloud->points[ i ].x;
				Yi( 1 ) = targetPointCloud->points[ index[ 0 ] ].y; Xi( 1 ) = modelPointCloud->points[ i ].y;
				Yi( 2 ) = targetPointCloud->points[ index[ 0 ] ].z; Xi( 2 ) = modelPointCloud->points[ i ].z;
				
				/* t - Yi */
				tYi = t - Yi;

				/* RXi + t - Yi */
				RXitYi = R * Xi + t - Yi;

				/* ���z�x�N�g�� */
				jacob( 0 ) = tYi.transpose() * dR[ 0 ] * Xi;
				jacob( 1 ) = tYi.transpose() * dR[ 1 ] * Xi;
				jacob( 2 ) = tYi.transpose() * dR[ 2 ] * Xi;
				jacob( 3 ) = RXitYi( 0 );
				jacob( 4 ) = RXitYi( 1 );
				jacob( 5 ) = RXitYi( 2 );

				/* �]���֐� C �̈�K���� */
				dfA += jacob;	// dfA = [ ��C/��r1, ��C/r2, ��C/r3, ��C/��t1, ��C/��t2, ��C/��t3 ]T
			}

			/* �w�b�Z�s�� */
			H = dfA * dfA.transpose();

			/* ���̃p�����[�^�x�N�g���̎Z�o */
			lambdaI = lambda * I.array();		// ��I
			lambdaIH = lambdaI + H;				// ��I + H
			lambdaIHinv = lambdaIH.inverse();	// ( ��I + H )-1
			nextA = A - lambdaIHinv * dfA;		// ���̃p�����[�^�x�N�g�� A
		}
		
		
		/* ���̕]���l C ���������ꂽ�ꍇ */
		else if( nextC >= C ){

			/* �ɂ̍X�V */
			lambda = lambda * 10;

			/* ���̃p�����[�^�x�N�g���̎Z�o */
			lambdaI = lambda * I.array();		// ��I
			lambdaIH = lambdaI + H;				// ��I + H
			lambdaIHinv = lambdaIH.inverse();	// ( ��I + H )-1
			nextA = A - lambdaIHinv * dfA;		// ���̃p�����[�^�x�N�g�� A
		}

		cout << "��: " << endl << lambda << endl;

		/* nextR, nextT �̎擾 */
		Rodrigues( 0 ) = nextA( 0 );
		Rodrigues( 1 ) = nextA( 1 );
		Rodrigues( 2 ) = nextA( 2 );
		PartialDerivativeForRodrigues( &Rodrigues, dR );	// ��]�̂R�v�f r1, r2, r3 �Ŕ����������ʂ��擾���AdR[ 3 ] �Ɋi�[
		nextR = ReconstructRFromRodrigues( &Rodrigues );	// Rodrigues �̉�]�x�N�g������̉�]�s��𕜌�
		nextT( 0 ) = nextA( 3 );
		nextT( 1 ) = nextA( 4 );
		nextT( 2 ) = nextA( 5 );


		/* ���̕]���l C �̎Z�o */
		nextC = 0;
		for( int i = 0; i < modelPointCloud->points.size(); i++ ){

			/* R, t �ɂ�郂�f���_���W�̕ϊ� */
			bp( 0 ) = modelPointCloud->points[ i ].x;
			bp( 1 ) = modelPointCloud->points[ i ].y;
			bp( 2 ) = modelPointCloud->points[ i ].z;
			ap = R * bp + t;
			transformedModelPoint.x = ap( 0 ); transformedModelPoint.r = modelPointCloud->points[ i ].r;
			transformedModelPoint.y = ap( 1 ); transformedModelPoint.g = modelPointCloud->points[ i ].g;
			transformedModelPoint.z = ap( 2 ); transformedModelPoint.b = modelPointCloud->points[ i ].b;

			/* KdTree �ɂ��Ή��_�T�� */
			kdtree.nearestKSearch( transformedModelPoint, 1, index, distance2 );	// ���f���_�Q�̊e�_����T���Ώۓ_�Q���̍ŋߓ_�܂ł̋����̎Z�o

			/* Yi �� Xi */
			Yi( 0 ) = targetPointCloud->points[ index[ 0 ] ].x; Xi( 0 ) = modelPointCloud->points[ i ].x;
			Yi( 1 ) = targetPointCloud->points[ index[ 0 ] ].y; Xi( 1 ) = modelPointCloud->points[ i ].y;
			Yi( 2 ) = targetPointCloud->points[ index[ 0 ] ].z; Xi( 2 ) = modelPointCloud->points[ i ].z;

			/* ���̕]���l C �̎Z�o */
			YiRXit = Yi - ( nextR * Xi + nextT );	// Yi - ( R * Xi + t )
			norm = YiRXit.norm();					// || Yi - ( R * Xi + t ) ||
			norm2 = pow( norm, 2 );					// || Yi - ( R * Xi + t ) || ��2��
			nextC += norm2;							// ���̕]���l C
		}


		/* ���[�v�񐔂̍X�V */
		loopCounter++;

		if( loopCounter >= LOOP_MAX ) break;	// ���[�v�񐔂�臒l�ɂ��œK���I��
	}





















#ifdef CUT

	/* �}�[�J�[�g�@�ɂ��œK���̃��[�v */
	while( 1 ){

		/* ���[�v�񐔂̃R�}���h���C���o�� */
		cout << loopCounter + 1 << endl;

		/* ���[�v���� */
		if( loopCounter == 0 ){

			/* Rodrigues �̉�]�x�N�g�� r = [ r1, r2, r3 ]T �̍쐬 */
			Rodrigues = ConvertRodrigues( &initR );

			/* ��]�s��̕Δ��� */
			PartialDerivativeForRodrigues( &Rodrigues, dR );	// ��]�̂R�v�f r1, r2, r3 �Ŕ����������ʂ��擾���AdR[ 3 ] �Ɋi�[

			/* R, t �̐ݒ� */
			R = ReconstructRFromRodrigues( &Rodrigues );	// Rodrigues �̉�]�x�N�g������̉�]�s��𕜌�
			t = initT;

			/* �p�����[�^�x�N�g�� A �̐ݒ� */
			A( 0 ) = Rodrigues( 0 );	// r1
			A( 1 ) = Rodrigues( 1 );	// r2
			A( 2 ) = Rodrigues( 2 );	// r3
			A( 3 ) = t( 0 );			// t1
			A( 4 ) = t( 1 );			// t2
			A( 5 ) = t( 2 );			// t3
		}

		/* ���[�v�񏉉� */
		if( loopCounter > 0 ){

			/* �}�[�J�[�g�@�ɂ��œK����̓_�Q */
			pcl::PointCloud< pcl::PointXYZRGB >::Ptr transformedModelPointCloud( new pcl::PointCloud< pcl::PointXYZRGB > );	// �ϊ����ꂽ���f���_�Q
			pcl::PointCloud< pcl::PointXYZRGB >::Ptr mergedPointCloud( new pcl::PointCloud< pcl::PointXYZRGB > );			// �������ꂽ�Q�_�Q

			/* ���� r1, r2, r3, t1, t2, t3 �̎擾 */
			Rodrigues( 0 ) = nextA( 0 );	// r1
			Rodrigues( 1 ) = nextA( 1 );	// r2
			Rodrigues( 2 ) = nextA( 2 );	// r3
			nextT( 0 ) = nextA( 3 );		// t1
			nextT( 1 ) = nextA( 4 );		// t2
			nextT( 2 ) = nextA( 5 );		// t3

			/* Rodrigues �̉�]�x�N�g������̉�]�s�� R �𕜌� */
			nextR = ReconstructRFromRodrigues( &Rodrigues );	// R
			nextC = 0;	// ���̕]���l nextC ��������

			for( int i = 0; i < modelPointCloud->points.size(); i++ ){

				/* R, t �ɂ�郂�f���_���W�̕ϊ� */
				bp( 0 ) = modelPointCloud->points[ i ].x;
				bp( 1 ) = modelPointCloud->points[ i ].y;
				bp( 2 ) = modelPointCloud->points[ i ].z;
				ap = R * bp + t;
				transformedModelPoint.x = ap( 0 ); transformedModelPoint.r = modelPointCloud->points[ i ].r;
				transformedModelPoint.y = ap( 1 ); transformedModelPoint.g = modelPointCloud->points[ i ].g;
				transformedModelPoint.z = ap( 2 ); transformedModelPoint.b = modelPointCloud->points[ i ].b;

				/* KdTree �ɂ��Ή��_�T�� */
				kdtree.nearestKSearch( transformedModelPoint, 1, index, distance2 );	// ���f���_�Q�̊e�_����T���Ώۓ_�Q���̍ŋߓ_�܂ł̋����̎Z�o

				/* Yi �� Xi */
				Yi( 0 ) = targetPointCloud->points[ index[ 0 ] ].x; Xi( 0 ) = modelPointCloud->points[ i ].x;
				Yi( 1 ) = targetPointCloud->points[ index[ 0 ] ].y; Xi( 1 ) = modelPointCloud->points[ i ].y;
				Yi( 2 ) = targetPointCloud->points[ index[ 0 ] ].z; Xi( 2 ) = modelPointCloud->points[ i ].z;

				/* ���̕]���l C �̎Z�o */
				YiRXit = Yi - ( nextR * Xi + nextT );	// Yi - ( R * Xi + t )
				norm = YiRXit.norm();					// || Yi - ( R * Xi + t ) ||
				norm2 = pow( norm, 2 );					// || Yi - ( R * Xi + t ) || ��2��
				nextC += norm2;							// ���̕]���l C

				/* R, t �ɂ��ϊ���̓_�Q�̍쐬 */
				transformedModelPointCloud->points.push_back( transformedModelPoint );
			}

			transformedModelPointCloud->width = modelPointCloud->points.size();
			transformedModelPointCloud->height = 1;

			/* 2�_�Q�̓��� */
			*mergedPointCloud = *transformedModelPointCloud + *targetPointCloud;

			/* 2�_�Q�̏o�� */
			if( pcl::io::savePLYFileASCII( OutputFileName, *mergedPointCloud ) == -1 ){ cout << "Caution!: PCD �f�[�^�̏������ݎ��s" << endl; }
		}
		
		/* ���̕]���l C �����݂̕]���l C �����������Ȃ����ꍇ or ���[�v�̏��� */
		if( nextC < C || loopCounter == 0 ){

			/* �]���l C, �p�����[�^�x�N�g�� A, �W���ɂ̍X�V */
			if( loopCounter > 0 ){
				
				absC = fabs( nextC - C );			// ���݂̕]���l C �Ǝ��̕]���l C �̍��̐�Βl�F| nextC - C |
				difA = nextA - A;
				norm = difA.norm();
				absA = fabs( norm );

				cout << "C: " << C << endl;
				cout << "nextC: " << nextC << endl;
				cout << "absC: " << absC << endl;
				cout << "absA: " << absA << endl;

				if( absC < THRESHOLD_C ) break;		// �]���l C �̎����ɂ��œK���I��
				if( absA < THRESHOLD_A ) break;		// �p�����[�^�x�N�g�� A �̎����ɂ��œK���I��

				C = nextC;							// ���݂̕]���l C ���X�V
				A = nextA;							// ���݂̃p�����[�^�x�N�g�� A ���X�V
				R = nextR;							// ���݂̉�]�s��̍X�V
				t = nextT;							// ���݂̕��i�x�N�g���̍X�V
				//nextA = Eigen::VectorXd::Zero( 6 );	// ���̃p�����[�^�x�N�g�� nextA ��������
				lambda = lambda * 0.1;				// �W���ɂ̒l�� 0.1 �{�F�j���[�g���@�̉e�����剻
				flag = 1;
			}

			/* ������ */
			H = Eigen::MatrixXd::Zero( 6, 6 );		// �w�b�Z�s��̑S�v�f�� 0 �ɏ�����
			I = Eigen::MatrixXd::Identity( 6, 6 );	// �P�ʍs��̍쐬
			dfA = Eigen::VectorXd::Zero( 6 );		// �]���֐� C �̈�K�����x�N�g���̑S�v�f�� 0 �ɏ�����
		
			/* �]���֐� C �̈�K�����̎Z�o */
			for( int i = 0; i < modelPointCloud->points.size(); i++ ){

				/* R, t �ɂ�郂�f���_���W�̕ϊ� */
				bp( 0 ) = modelPointCloud->points[ i ].x;
				bp( 1 ) = modelPointCloud->points[ i ].y;
				bp( 2 ) = modelPointCloud->points[ i ].z;
				ap = R * bp + t;
				transformedModelPoint.x = ap( 0 ); transformedModelPoint.r = modelPointCloud->points[ i ].r;
				transformedModelPoint.y = ap( 1 ); transformedModelPoint.g = modelPointCloud->points[ i ].g;
				transformedModelPoint.z = ap( 2 ); transformedModelPoint.b = modelPointCloud->points[ i ].b;

				/* KdTree �ɂ��Ή��_�T�� */
				kdtree.nearestKSearch( transformedModelPoint, 1, index, distance2 );	// ���f���_�Q�̊e�_����T���Ώۓ_�Q���̍ŋߓ_�܂ł̋����̎Z�o

				/* Yi �� Xi */
				Yi( 0 ) = targetPointCloud->points[ index[ 0 ] ].x; Xi( 0 ) = modelPointCloud->points[ i ].x;
				Yi( 1 ) = targetPointCloud->points[ index[ 0 ] ].y; Xi( 1 ) = modelPointCloud->points[ i ].y;
				Yi( 2 ) = targetPointCloud->points[ index[ 0 ] ].z; Xi( 2 ) = modelPointCloud->points[ i ].z;

				/* ���񎞂ɂ�����]���l C �̎Z�o */
				if( loopCounter == 0 ){
					YiRXit = Yi - ( R * Xi + t );	// Yi - ( R * Xi + t )
					norm = YiRXit.norm();			// || Yi - ( R * Xi + t ) ||
					norm2 = norm * norm;			// || Yi - ( R * Xi + t ) || ��2��
					C += norm2;						// �]���֐� C
				}

				/* t - Yi */
				tYi = t - Yi;

				/* RXi + t - Yi */
				RXitYi = R * Xi + t - Yi;

				/* ���z�x�N�g�� */
				jacob( 0 ) = tYi.transpose() * dR[ 0 ] * Xi;
				jacob( 1 ) = tYi.transpose() * dR[ 1 ] * Xi;
				jacob( 2 ) = tYi.transpose() * dR[ 2 ] * Xi;
				jacob( 3 ) = RXitYi( 0 );
				jacob( 4 ) = RXitYi( 1 );
				jacob( 5 ) = RXitYi( 2 );

				/* �]���֐� C �̈�K���� */
				dfA += jacob;	// dfA = [ ��C/��r1, ��C/r2, ��C/r3, ��C/��t1, ��C/��t2, ��C/��t3 ]T

				/* Jacob * JacobT */
				//jacob2 = jacob * jacob.transpose();

				/* �w�b�Z�s�� */
				//H += jacob2;
			}

			/* �w�b�Z�s��̎Z�o */
			H = dfA * dfA.transpose();

			/* ���̃p�����[�^�x�N�g���̎Z�o */
			lambdaI = lambda * I.array();		// ��I
			lambdaIH = lambdaI + H;				// ��I + H
			lambdaIHinv = lambdaIH.inverse();	// ( ��I + H )-1
			nextA = A - lambdaIHinv * dfA;		// ���̃p�����[�^�x�N�g�� A
		}
		
		cout << "C: " << C << endl;
		cout << "nextC: " << nextC << endl;

		/* ���̕]���l C �����݂̕]���l C �����傫���Ȃ����ꍇ & ���[�v�񏉉� */
		if( nextC >= C && loopCounter > 0 && flag == 0 ){

			/* �ɂ̒l���������ύX */
			lambda = lambda * 10;	// �W���ɂ̒l�� 10 �{�F���z�@�̉e�����剻

			/* ���̃p�����[�^�x�N�g���̎Z�o */
			lambdaI = lambda * I.array();		// ��I
			lambdaIH = lambdaI + H;				// ��I + H
			lambdaIHinv = lambdaIH.inverse();	// ( ��I + H )-1
			nextA = A - lambdaIHinv * dfA;		// ���̃p�����[�^�x�N�g�� A
		}

		cout << "��: " << lambda << endl;
		
		flag = 0;

		/* ���[�v�J�E���^�̍X�V */
		loopCounter++;

		if( loopCounter >= LOOP_MAX ) break;	// ���[�v�񐔂�臒l�ɂ��œK���I��
	}
#endif


	/*** �v���O�����I���̃R�[�� ***/
	cout << "�y �v���O�����I�� �z" << endl;


	return 0;
}