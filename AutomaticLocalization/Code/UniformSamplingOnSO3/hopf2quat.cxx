/***** 回転空間の均等なサンプリング( ホップ座標の出力関数 ) *****/


/*** インクルードファイル ***/

/* 作成ヘッダファイル */
#include"so3_sequence.h"


/*** ホップ座標の Output 関数 ***/
bool hopf2quat( vector < vector <double> > Points ){


	/* 変数の定義 */
	double x1 = 0, x2 = 0, x3 = 0, x4 = 0;	// 四元数
	ofstream output1;						// ファイル出力用変数
	ofstream output2;						// ファイル出力用変数
	ofstream output3;						// ファイル出力用変数
	ofstream output4;						// ファイル出力用変数
	ofstream output5;						// ファイル出力用変数
	output1.open( "Output/SamplingR/quaternion.qua" );		// 出力ファイルの指定 & 出力用ファイルの展開
	output2.open( "Output/SamplingR/quaternion.csv" );		// 出力ファイルの指定 & 出力用ファイルの展開
	output3.open( "Output/SamplingR/quaternion.txt" );		// 出力ファイルの指定 & 出力用ファイルの展開
	output4.open( "Output/SamplingR/degreeθΦΨ.csv" );		// 出力ファイルの指定 & 出力用ファイルの展開
	output5.open( "Output/SamplingR/radianθΦΨ.csv" );		// 出力ファイルの指定 & 出力用ファイルの展開

	/* 点群数分だけループ */
	for( int i = 0; i < Points.size(); i++ ){

		/* ホップ座標を構成する四元数 */
		// θ ( 球面座標 S2, 単位：ラジアン ) … Points[ i ][ 0 ]
		// Φ ( 球面座標 S2, 単位：ラジアン ) … Points[ i ][ 1 ]
		// Ψ ( 回転平面 S1, 単位：ラジアン ) … Points[ i ][ 2 ]
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

	/* メモリの解放 */
	output1.close();
	output2.close();
	output3.close();
	output4.close();
	output5.close();

	return true;
}
