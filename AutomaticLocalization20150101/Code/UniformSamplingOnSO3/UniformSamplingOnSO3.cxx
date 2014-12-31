/***** ��]��Ԃ̋ϓ��ȃT���v�����O *****/


/* * * * * �}�j���A�� * * * * */
// �v���O�������s��A�쐬�������_�̐������
// �� ��F 72( Base Resolution ), 576( Resolution Level 1 )
//         4608( Resolution Level 2 ), 36864( Resolution Level 3 )


/*** �C���N���[�h�t�@�C�� ***/

/* ���̑� */
#include"so3_sequence.h"


/*** �萔�̒�` ***/

/* �x�[�X�_�Q�̏����z�񂪋L�q���ꂽ�t�@�C���� */
char inputSequenceFileName[] = "Code/UniformSamplingOnSO3/seq.txt";


/*** ���C���֐� ***/
int main(){


	/* �ϐ��̒�` */
	long int num_points = 0;				// �쐬����T���v�����O�_��
	vector < double > Point;				// �T���v�����O�_�p�x�N�g���ϐ�
	vector < vector<double> > Points;		// �T���v�����O�_�p�x�N�g���ϐ�
	vector < vector<int> > Sequence_base;
	vector <int> temp;
	ifstream input;				// �t�@�C���p�ϐ�
	long int base_grid = 0;		// �x�[�X�O���b�h�̒ʂ��ԍ�
	long int cur_point = 0;		// �𑜓x���x��
	long int point_healpix = 0;	// 2�������� S2 ��̓_ [ HEALPix ]
	double point_S1 = 0;		// ��]���� S1 �̃T���v�����O�p(��)
	double theta = 0;			// ���ʍ��W S2 �̉�]�p( ���W�A�� )
	double phi = 0;				// ���ʍ��W S2 �̉�]�p( ���W�A�� )
	double psi = 0;				// ��]���� S1 �̉�]�p( ���W�A�� )
	int limit = 0;				// �x�[�X�O���b�h�_�̏���� < 72

	
	/*** ���� ***/

	/* �쐬�������T���v�����O�_���̎w�� */
	cout << "Enter number of points in the sequence: ";
	cin >> num_points;

	
	/*** �x�[�X�_�Q�̏����z�񂪋L�q���ꂽ�t�@�C���̓ǂݍ��� ***/

	/* �t�@�C���̓W�J */
	input.open( inputSequenceFileName );	// �w�肳�ꂽ�t�@�C���̓W�J
	
	/* �������̊m�� */
	Sequence_base.resize( 0 );
	temp.resize( 2 );

	/* �t�@�C�����e�̓ǂݍ��� */
	while( !input.eof() ){

		/* �t�@�C������z��ԍ���ǂݍ��� */
		input >> temp[ 0 ] >> temp[ 1 ];	// 1�����x�N�g���Ɋi�[

		/* 2�����z��ւ�1�����z��̒ǉ� */
		Sequence_base.push_back( temp );	// �x�N�g���̖����ɑ�1������ǉ�����
	}

	/* �������̉�� */
	input.close();

	/* �x�N�g���̍ŏI�v�f�̍폜 */
	Sequence_base.pop_back();
	

	/*** �x�[�X�O���b�h�_�̐ݒ�F���͂��ꂽ���߂� 72 �_���x�[�X�O���b�h�_�ɑI�� ***/
	//first seventy two points are the base grid points;
	Points.resize( 0 );
	if( num_points < 72 ){
		limit = num_points;
	}else{
		limit = 72;
	}


	/*** �x�[�X�𑜓x�̓_�Q����72�_�ȉ��̏ꍇ ***/
	for( int i = 0; i < limit; i++ ){

		/* �������̊m�� */
		Point.resize( 0 );

		pix2ang_nest( 1, Sequence_base[ i ][ 0 ], &theta, &phi );
		
		/* ��]���� S1 �ւ̔ԍ��̊���U��( 60���Ԋu ) */
		// mapping index on S1 to its angle value
		switch( Sequence_base[ i ][ 1 ] ){

			case 0:
				point_S1 = 30;
				break;
			case 1:
				point_S1 = 90;
				break;
			case 2: 
				point_S1 = 150;
				break;
			case 3:
				point_S1 = 210;
				break;
			case 4: 
				point_S1 = 270;
				break;
			case 5: 
				point_S1 = 330;
				break;
		}

		/* ��]���ʂ̉�]�p �� */
		psi = point_S1 * M_PI / 180;

		/* �x�N�g��( 1�����z�� )�̖����ɒl��ǉ� */
		Point.push_back( theta );	// ��]�p��
		Point.push_back( phi );		// ��]�p��
		Point.push_back( psi );		// ��]�p��
		Points.push_back( Point );	// 2�����z��̖�����1�����z���ǉ�
	}

	
	/*** �x�[�X�𑜓x�̓_�Q����72�_�ȏ�̏ꍇ ***/
	// this will only be called if points are more than 72.
	for( int i = 0; i < num_points - 72; i++ ){

		/* �������̊m�� */
		Point.resize( 0 );

		/* ������ */
		base_grid = i % 72;	// �x�[�X�O���b�h�̒ʂ��ԍ�
		cur_point = i / 72;	// �𑜓x���x���̎Z�o
		point_healpix = 4 * Sequence_base[ base_grid ][ 0 ];

		/* ��]����S1�ւ̔ԍ��̊���U�� */
		// mapping index on S1 to its angle value
		switch( Sequence_base[ base_grid ][ 1 ] ){
			
			case 0:
				point_S1 = 30;
				break;
			case 1:
				point_S1 = 90;
				break;
			case 2: 
				point_S1 = 150;
				break;
			case 3:
				point_S1 = 210;
				break;
			case 4: 
				point_S1 = 270;
				break;
			case 5: 
				point_S1 = 330;
				break;
		}


		Point = find_point( Sequence_base[ base_grid ][ 0 ], cur_point, 1, point_healpix, point_S1 );
		//current point value, level, current point in healpix, current point for S1
		// ��1���� �c 
		// ��2���� �c �𑜓x���x�� 0, 1, 2, �c
		// ��3���� �c 
		// ��4���� �c 2�������� S2 ��̓_ [ HEALPix ]
		// ��5���� �c ��]���� S1 ��̓_

		/* 2�����z��̖�����1�����z���ǉ� */
		Points.push_back( Point );
	}
	

	/*** �T���v�����O�_�Q�̒P�ʎl�����ւ̕ϊ��ƌ��ʏo�� ***/
	if( hopf2quat( Points ) ){

		cout << ">>> Converting to quaternions is success" << endl;
		return 0;
	}else{
		
		cout << ">>> Problem in converting to quaternions" << endl;
		return 0;
	}
}
	
