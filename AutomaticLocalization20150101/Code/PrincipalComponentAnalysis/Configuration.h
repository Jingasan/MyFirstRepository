/***** �ݒ�w�b�_�t�@�C�� *****/


/*** �t���O�̊Ǘ� ***/
#define COMMENT	// �R�����g�̃R�}���h���C���o�� ON�F�o��, OFF�F��o��
#define MERGE	// �_�Q�����K����̃��f���_�Q�̓��� ON�F����, OFF�F�񓝍�
#define VIEWER	// �_�Q�̕\�� ON�F�\��, OFF�F��\��


/*** �萔�̒�` ***/

/* ���͂��郂�f���_�Q�t�@�C���� */
#define INPUTPOINTCLOUDNUM 30

/* ���f���_�Q�t�@�C���̐� */
#define POINTCLOUDNUM 30

/* ���͓_�Q�t�@�C���� */
const char InputFileName[ POINTCLOUDNUM ][ 128 ] = {
	{ "Input/PointData/ModelData/LiverR20/0001.pcd" },
	{ "Input/PointData/ModelData/LiverR20/0002.pcd" },
	{ "Input/PointData/ModelData/LiverR20/0003.pcd" },
	{ "Input/PointData/ModelData/LiverR20/0004.pcd" },
	{ "Input/PointData/ModelData/LiverR20/0005.pcd" },
	{ "Input/PointData/ModelData/LiverR20/0006.pcd" },
	{ "Input/PointData/ModelData/LiverR20/0007.pcd" },
	{ "Input/PointData/ModelData/LiverR20/0008.pcd" },
	{ "Input/PointData/ModelData/LiverR20/0009.pcd" },
	{ "Input/PointData/ModelData/LiverR20/0010.pcd" },
	{ "Input/PointData/ModelData/LiverR20/0011.pcd" },
	{ "Input/PointData/ModelData/LiverR20/0012.pcd" },
	{ "Input/PointData/ModelData/LiverR20/0013.pcd" },
	{ "Input/PointData/ModelData/LiverR20/0014.pcd" },
	{ "Input/PointData/ModelData/LiverR20/0015.pcd" },
	{ "Input/PointData/ModelData/LiverR20/0016.pcd" },
	{ "Input/PointData/ModelData/LiverR20/0017.pcd" },
	{ "Input/PointData/ModelData/LiverR20/0018.pcd" },
	{ "Input/PointData/ModelData/LiverR20/0019.pcd" },
	{ "Input/PointData/ModelData/LiverR20/0020.pcd" },
	{ "Input/PointData/ModelData/LiverR20/0021.pcd" },
	{ "Input/PointData/ModelData/LiverR20/0022.pcd" },
	{ "Input/PointData/ModelData/LiverR20/0023.pcd" },
	{ "Input/PointData/ModelData/LiverR20/0024.pcd" },
	{ "Input/PointData/ModelData/LiverR20/0025.pcd" },
	{ "Input/PointData/ModelData/LiverR20/0026.pcd" },
	{ "Input/PointData/ModelData/LiverR20/0027.pcd" },
	{ "Input/PointData/ModelData/LiverR20/0028.pcd" },
	{ "Input/PointData/ModelData/LiverR20/0029.pcd" },
	{ "Input/PointData/ModelData/LiverR20/0030.pcd" }
};

/* �o�͓_�Q�t�@�C���� */
const char OutputPLYFileName1[] = "Output/PrincipalComponentAnalysis/merged30PointCloud.ply";