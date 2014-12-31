/***** ��]��Ԃ̋ϓ��ȃT���v�����O( �z�b�v���W�̏o�͊֐� ) *****/


/*** �C���N���[�h�t�@�C�� ***/

/* �쐬�w�b�_�t�@�C�� */
#include"so3_sequence.h"


/*** �z�b�v���W�� Output �֐� ***/
bool hopf2quat( vector < vector <double> > Points ){


	/* �ϐ��̒�` */
	double x1 = 0, x2 = 0, x3 = 0, x4 = 0;	// �l����
	ofstream output1;						// �t�@�C���o�͗p�ϐ�
	ofstream output2;						// �t�@�C���o�͗p�ϐ�
	ofstream output3;						// �t�@�C���o�͗p�ϐ�
	ofstream output4;						// �t�@�C���o�͗p�ϐ�
	ofstream output5;						// �t�@�C���o�͗p�ϐ�
	output1.open( "Output/SamplingR/quaternion.qua" );		// �o�̓t�@�C���̎w�� & �o�͗p�t�@�C���̓W�J
	output2.open( "Output/SamplingR/quaternion.csv" );		// �o�̓t�@�C���̎w�� & �o�͗p�t�@�C���̓W�J
	output3.open( "Output/SamplingR/quaternion.txt" );		// �o�̓t�@�C���̎w�� & �o�͗p�t�@�C���̓W�J
	output4.open( "Output/SamplingR/degree�ƃ���.csv" );		// �o�̓t�@�C���̎w�� & �o�͗p�t�@�C���̓W�J
	output5.open( "Output/SamplingR/radian�ƃ���.csv" );		// �o�̓t�@�C���̎w�� & �o�͗p�t�@�C���̓W�J

	/* �_�Q�����������[�v */
	for( int i = 0; i < Points.size(); i++ ){

		/* �z�b�v���W���\������l���� */
		// �� ( ���ʍ��W S2, �P�ʁF���W�A�� ) �c Points[ i ][ 0 ]
		// �� ( ���ʍ��W S2, �P�ʁF���W�A�� ) �c Points[ i ][ 1 ]
		// �� ( ��]���� S1, �P�ʁF���W�A�� ) �c Points[ i ][ 2 ]
		x4 = sin( Points[ i ][ 0 ] / 2 ) * sin( Points[ i ][ 1 ] + Points[ i ][ 2 ] / 2 );
		x1 = cos( Points[ i ][ 0 ] / 2 ) * cos( Points[ i ][ 2 ] / 2 );
		x2 = cos( Points[ i ][ 0 ] / 2 ) * sin( Points[ i ][ 2 ] / 2 );
		x3 = sin( Points[ i ][ 0 ] / 2 ) * cos( Points[ i ][ 1 ] + Points[ i ][ 2 ] / 2 );
		output1 << x1 << "\t" << x2 << "\t" << x3 << "\t" << x4 << endl;
		output2 << x1 << "," << x2 << "," << x3 << "," << x4 << endl;
		output3 << x1 << "\t" << x2 << "\t" << x3 << "\t" << x4 << endl;
		
		output4 << Points[ i ][ 0 ] * 180 / M_PI << "," << Points[ i ][ 1 ] * 180 / M_PI << "," << Points[ i ][ 2 ] * 180 / M_PI << endl;
		output5 << Points[ i ][ 0 ] << "," << Points[ i ][ 1 ] << "," << Points[ i ][ 2 ] << endl;
	}

	/* �������̉�� */
	output1.close();
	output2.close();
	output3.close();
	output4.close();
	output5.close();

	return true;
}
