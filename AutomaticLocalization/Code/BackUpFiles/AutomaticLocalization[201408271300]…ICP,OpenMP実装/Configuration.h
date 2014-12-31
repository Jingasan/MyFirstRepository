/***** �ݒ�w�b�_�t�@�C�� *****/
// PCL �x����\���R�[�h�F4996;4819;4503;4305;4244;4005;


/*** �t���O�̒�` ***/
#define COMMENT					// �R�}���h���C���o�� ON:�o�� OFF:��o��
#define VISUALIZE				// �_�Q�̉���
#define ESRESULT				// �S�T�����ʂ̏o��
#define ITERATIVECLOSESTPOINT	// ICP �ɂ��œK�� ON: ICP ���s OFF: ICP ����s 
#define ICPRESULT				// ICP ���ʂ̏o��


/*** �萔�̒�` ***/

/* createDistanceField �֐� */
#define INFINITY 99999			// DF �쐬���̏�����
#define TOMETER 1000			// 3�����_�Q���W�̒P�ʕϊ��F m �� mm
#define DISTANCEFIELDSIZE 100	// DF �̕���\�̊���l

/* �S�T�� */
#define LIMIT_X 2			// ���i X �̒T���͈͂̍i�荞�݁F����l / LIMIT_X, �����l / LIMIT_X
#define LIMIT_Y 2			// ���i Y �̒T���͈͂̍i�荞�݁F����l / LIMIT_Y, �����l / LIMIT_X
#define LIMIT_Z 2			// ���i Z �̒T���͈͂̍i�荞�݁F����l / LIMIT_Z, �����l / LIMIT_X
#define SAMP_R_LEVEL 1		// ��]��Ԃ̋ϓ��ȃT���v�����O�̉𑜓x���x���F 0, 1, 2, 3
#define SAMP_X 10			// ���i X �̃T���v�����O��
#define SAMP_Y 10			// ���i Y �̃T���v�����O��
#define SAMP_Z 10			// ���i Z �̃T���v�����O��
//#define SAMP_THETA 10		//
//#define SAMP_PHI 10			//
//#define SAMP_PSI 10			//
//#define DISTANCE_ERROR 10	// ���f���_�Q�ƒT���Ώۓ_�Q��1�_������� DF �]���l��臒l�F�]���l�����̒l�����������Ȃ� R �� t �����Ƃ��Ď擾

/* ICP */
#define ICP_LOOP_MAX 200


/*** ���̓t�@�C���Q ***/

/* �T���v�����O���ꂽ��]�s��̊�ƂȂ�l�����Ɖ�]�p�ƃ��� */
const char QuaternionFileName1[] = "Output/SamplingR/72quaternion.csv";
const char QuaternionFileName2[] = "Output/SamplingR/576quaternion.csv";
const char QuaternionFileName3[] = "Output/SamplingR/4608quaternion.csv";
const char QuaternionFileName4[] = "Output/SamplingR/36864quaternion.csv";
const char DegreeFileName1[] = "Output/SamplingR/72degree.csv";
const char DegreeFileName2[] = "Output/SamplingR/576degree.csv";
const char DegreeFileName3[] = "Output/SamplingR/4608degree.csv";
const char DegreeFileName4[] = "Output/SamplingR/36864degree.csv";


/*** �o�̓t�@�C���Q ***/

/* �S�T���̑O�㌋�� */
const char BeforeExhaustiveSearchPCDFileName[] = "Output/ExhaustiveSearchResult/MergedPointCloudDataBefore.pcd";	// �S�T���O��2�_�Q�̓������� 
const char BeforeExhaustiveSearchPLYFileName[] = "Output/ExhaustiveSearchResult/MergedPointCloudDataBefore.ply";	// �S�T���O��2�_�Q�̓�������
const char AfterExhaustiveSearchPCDFileName[] = "Output/ExhaustiveSearchResult/MergedPointCloudDataAfter.pcd";		// �S�T�����2�_�Q�̓�������
const char AfterExhaustiveSearchPLYFileName[] = "Output/ExhaustiveSearchResult/MergedPointCloudDataAfter.ply";		// �S�T�����2�_�Q�̓�������

/* ICP �A���S���Y����̌��� */
const char IcpRtFileName[] = "Output/ICPResult/ICP_Rt.csv";
const char IcpRMSEFileName[] = "Output/ICPResult/ICP_RMSE.csv";
const char IcpPCDFileName[] = "Output/ICPResult/MergedPointCloudDataAfterICP.pcd";	// ICP ��p�����œK�����2�_�Q�̓�������
const char IcpPLYFileName[] = "Output/ICPResult/MergedPointCloudDataAfterICP.ply";	// ICP ��p�����œK�����2�_�Q�̓�������