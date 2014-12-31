/***** 回転空間の均等なサンプリング *****/


/* * * * * マニュアル * * * * */
// プログラム実行後、作成したい点の数を入力
// ⇒ 例： 72( Base Resolution ), 576( Resolution Level 1 )
//         4608( Resolution Level 2 ), 36864( Resolution Level 3 )


/*** インクルードファイル ***/

/* その他 */
#include"so3_sequence.h"


/*** 定数の定義 ***/

/* ベース点群の初期配列が記述されたファイル名 */
char inputSequenceFileName[] = "Code/UniformSamplingOnSO3/seq.txt";


/*** メイン関数 ***/
int main(){


	/* 変数の定義 */
	long int num_points = 0;				// 作成するサンプリング点数
	vector < double > Point;				// サンプリング点用ベクトル変数
	vector < vector<double> > Points;		// サンプリング点用ベクトル変数
	vector < vector<int> > Sequence_base;
	vector <int> temp;
	ifstream input;				// ファイル用変数
	long int base_grid = 0;		// ベースグリッドの通し番号
	long int cur_point = 0;		// 解像度レベル
	long int point_healpix = 0;	// 2次元球面 S2 上の点 [ HEALPix ]
	double point_S1 = 0;		// 回転平面 S1 のサンプリング角(°)
	double theta = 0;			// 球面座標 S2 の回転角( ラジアン )
	double phi = 0;				// 球面座標 S2 の回転角( ラジアン )
	double psi = 0;				// 回転平面 S1 の回転角( ラジアン )
	int limit = 0;				// ベースグリッド点の上限数 < 72

	
	/*** 入力 ***/

	/* 作成したいサンプリング点数の指定 */
	cout << "Enter number of points in the sequence: ";
	cin >> num_points;

	
	/*** ベース点群の初期配列が記述されたファイルの読み込み ***/

	/* ファイルの展開 */
	input.open( inputSequenceFileName );	// 指定されたファイルの展開
	
	/* メモリの確保 */
	Sequence_base.resize( 0 );
	temp.resize( 2 );

	/* ファイル内容の読み込み */
	while( !input.eof() ){

		/* ファイルから配列番号を読み込む */
		input >> temp[ 0 ] >> temp[ 1 ];	// 1次元ベクトルに格納

		/* 2次元配列への1次元配列の追加 */
		Sequence_base.push_back( temp );	// ベクトルの末尾に第1引数を追加する
	}

	/* メモリの解放 */
	input.close();

	/* ベクトルの最終要素の削除 */
	Sequence_base.pop_back();
	

	/*** ベースグリッド点の設定：入力された初めの 72 点をベースグリッド点に選択 ***/
	//first seventy two points are the base grid points;
	Points.resize( 0 );
	if( num_points < 72 ){
		limit = num_points;
	}else{
		limit = 72;
	}


	/*** ベース解像度の点群数が72点以下の場合 ***/
	for( int i = 0; i < limit; i++ ){

		/* メモリの確保 */
		Point.resize( 0 );

		pix2ang_nest( 1, Sequence_base[ i ][ 0 ], &theta, &phi );
		
		/* 回転平面 S1 への番号の割り振り( 60°間隔 ) */
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

		/* 回転平面の回転角 Ψ */
		psi = point_S1 * M_PI / 180;

		/* ベクトル( 1次元配列 )の末尾に値を追加 */
		Point.push_back( theta );	// 回転角θ
		Point.push_back( phi );		// 回転角Φ
		Point.push_back( psi );		// 回転角Ψ
		Points.push_back( Point );	// 2次元配列の末尾に1次元配列を追加
	}

	
	/*** ベース解像度の点群数が72点以上の場合 ***/
	// this will only be called if points are more than 72.
	for( int i = 0; i < num_points - 72; i++ ){

		/* メモリの確保 */
		Point.resize( 0 );

		/* 初期化 */
		base_grid = i % 72;	// ベースグリッドの通し番号
		cur_point = i / 72;	// 解像度レベルの算出
		point_healpix = 4 * Sequence_base[ base_grid ][ 0 ];

		/* 回転平面S1への番号の割り振り */
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
		// 第1引数 … 
		// 第2引数 … 解像度レベル 0, 1, 2, …
		// 第3引数 … 
		// 第4引数 … 2次元球面 S2 上の点 [ HEALPix ]
		// 第5引数 … 回転平面 S1 上の点

		/* 2次元配列の末尾に1次元配列を追加 */
		Points.push_back( Point );
	}
	

	/*** サンプリング点群の単位四元数への変換と結果出力 ***/
	if( hopf2quat( Points ) ){

		cout << ">>> Converting to quaternions is success" << endl;
		return 0;
	}else{
		
		cout << ">>> Problem in converting to quaternions" << endl;
		return 0;
	}
}
	
