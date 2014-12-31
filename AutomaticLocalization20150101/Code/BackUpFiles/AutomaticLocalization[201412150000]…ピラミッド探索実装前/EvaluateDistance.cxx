/***** �T���Ώۓ_�Q�ƃ��f���_�Q�Ƃ̋����]�� *****/


/*** �C���N���[�h�t�@�C�� ***/

/* �ݒ�p�w�b�_�t�@�C�� */
#include "Configuration.h"
#include "createDistanceField.h"


/*** ���O��Ԃ̐錾 ***/

/* C++ */
using namespace std;


/*** �T���Ώۓ_�Q�ƃ��f���_�Q�Ƃ̋����]�� ***/
void EvaluateDistance(
	int** DistanceField,						// DF
	struct ModelInformation* modelinformation,	// �T���Ώۓ_�Q�̊e����
	int* SumofDistance,							// �T���Ώۓ_�Q�ƃ��f���_�Q�Ƃ̋����]���l
	int* minSumofDistance,						// �T���Ώۓ_�Q�ƃ��f���_�Q�Ƃ̋����]���l�̍ŏ��l
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloud	// ���f���_�Q
){
	
	/* �ϐ��̒�` */
	int iSize = modelinformation->iSize;			// DF �� i �����̃T�C�Y
	int jSize = modelinformation->jSize;			// DF �� j �����̃T�C�Y
	int kSize = modelinformation->kSize;			// DF �� k �����̃T�C�Y
	int ijSize = iSize * jSize;						// DF �� i, j �����̖ʐ�
	double targetXSize = modelinformation->deltaX;	// �T���Ώۓ_�Q�� X �����̍ő啝 
	double targetYSize = modelinformation->deltaY;	// �T���Ώۓ_�Q�� Y �����̍ő啝
	double targetZSize = modelinformation->deltaZ;	// �T���Ώۓ_�Q�� Z �����̍ő啝
	double gravX = modelinformation->gravX;	// �T���Ώۂ̏d�S X
	double gravY = modelinformation->gravY;	// �T���Ώۂ̏d�S Y
	double gravZ = modelinformation->gravZ;	// �T���Ώۂ̏d�S Z
	int i, j, k;	// ���f���_�Q�� DF ���W�ɒ������l
	int sum = 0;	// �T���Ώۓ_�Q�ƃ��f���_�Q�Ƃ̋����]���l�̍��v

	
	/*** �T���Ώۓ_�Q�ƃ��f���_�Q�Ƃ̋����]���l�̎Z�o ***/
	for( int n = 0; n < cloud->size(); n++ ){

		/* ���f���_�Q�� DF ���ɂ���T���Ώۓ_�Q�̍��W�n�ֈړ� */
		cloud->points[ n ].x = cloud->points[ n ].x * TOMETER - gravX;
		cloud->points[ n ].y = cloud->points[ n ].y * TOMETER - gravY;
		cloud->points[ n ].z = cloud->points[ n ].z * TOMETER - gravZ;

		/* ���f���_�Q�� DF ��ł̍��W�l�̎Z�o */
		i = (int)( ( (double)iSize * ( cloud->points[ n ].x + targetXSize ) / ( 2.0 * targetXSize ) ) );//+ 0.5 );
		j = (int)( ( (double)jSize * ( cloud->points[ n ].y + targetYSize ) / ( 2.0 * targetYSize ) ) );//+ 0.5 );
		k = (int)( ( (double)kSize * ( cloud->points[ n ].z + targetZSize ) / ( 2.0 * targetZSize ) ) );//+ 0.5 );

#ifdef CUT
		i = (int)( (double)( ( (double)iSize * cloud->points[ n ].x ) / ( 2.0 * targetXSize ) ) + ( (double)iSize / 2.0 ) + 0.5 );
		j = (int)( (double)( ( (double)jSize * cloud->points[ n ].y ) / ( 2.0 * targetYSize ) ) + ( (double)jSize / 2.0 ) + 0.5 );
		k = (int)( (double)( ( (double)kSize * cloud->points[ n ].z ) / ( 2.0 * targetZSize ) ) + ( (double)kSize / 2.0 ) + 0.5 );
#endif

		/* �T���Ώۓ_�Q�ƃ��f���_�Q�Ƃ̋����]���l�̍��v�l���Z�o */
		sum += ( *DistanceField )[ k * ijSize + j * iSize + i ];

		/* �ŏ����v�����]���l�������݂̍��v�����]���l�̕����傫���ꍇ */
		if( sum > *minSumofDistance ) break;	// ���[�v�𔲂���
	}


#ifdef COMMENTa
	cout << "DFSize: " << iSize << " " << jSize << " " << kSize << endl;
	cout << "TargetSize: " << targetXSize << " " << targetYSize << " " << targetZSize << endl;
	cout << "TargetGravity: " << gravX << " " << gravY << " " << gravZ << endl;
	cout << "SumofDistance: " << sum << endl;
#endif


	/* �T���Ώۓ_�Q�ƃ��f���_�Q�Ƃ̋����]���l�̍��v */
	*SumofDistance = sum;
}