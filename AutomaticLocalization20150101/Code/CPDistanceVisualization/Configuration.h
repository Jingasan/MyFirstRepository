/***** �ݒ�w�b�_�t�@�C�� *****/


/*** �t���O�̒�` ***/
#define MULTIPLY_NUM 10	// �Ή��_�Ԃ̋����l�����̒l�{����F100�ŏ������ʂ܂Ŏc�� 


/*** �萔�̒�` ***/

/* ���̓t�@�C���p�X */
//const char CPDistanceFileName[] = "Output/ExhaustiveSearchResult/CPDistance.csv";		// �S�T����̑Ή��_(�ŋߓ_)�Ԃ̋���
const char CPDistanceFileName[] = "Output/ICPResult/CPDistance.csv";	// ICP�ɂ��œK����̑Ή��_(�ŋߓ_)�Ԃ̋���

/* �o�̓t�@�C���p�X */
//const char HistogramFileName[] = "Output/Histogram/AfterExhaustiveSearch/CPDistanceAfterES.csv";		// �q�X�g�O�����̃r���̏o�͐�G�N�Z���t�@�C����(�S�T����)
const char HistogramFileName[] = "Output/Histogram/AfterICP/CPDistanceAfterICP.csv";	// �q�X�g�O�����̃r���̏o�͐�G�N�Z���t�@�C����(�œK����)
//const char HistogramImageFileName[] = "Output/Histogram/AfterExhaustiveSearch/CPDistanceAfterES.png";	// �q�X�g�O�����摜(�S�T����)
const char HistogramImageFileName[] = "Output/Histogram/AfterICP/CPDistanceAfterICP.png";	// �q�X�g�O�����摜(�œK����)
