/***** 対応点間の距離のヒストグラムの作成 *****/
// 対応点間の距離が記録されたエクセルファイルを読み込み、ヒストグラムを作成・出力する
// ※ ヒストグラムの描画には、エクセルを使用すること


/*** インクルード ***/

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

/* 設定ヘッダファイル */
#include "Configuration.h"


/*** 名前空間の宣言 ***/

/* C++ */
using namespace std;


/*** メイン処理関数 ***/
int main( int argc, char **argv ){


	/* 変数の宣言 */
	FILE *inputFile;					// ファイル型変数
	int index;							// 対応点のインデックス
	float distance;						// 対応点間の距離
	float distanceInt;					// 対応点間の距離( int 型で MULTIPLY_NUM 倍後 )
	vector< float > distances;			// 対応点間の距離の配列
	vector< int > distancesInt;			// 対応点間の距離の配列( int 型で MULTIPLY_NUM 倍後 )
	float meanCPDistance = 0;			// 対応点間の距離の平均値
	float rmse = 0;						// RMSE( root mean squared error )：平均二乗誤差の平方根
	float maxCPDistance = -FLT_MAX;		// 対応点間の距離の最大値
	float minCPDistance = FLT_MAX;		// 対応点間の距離の最小値 
	int maxCPDistanceInt = -INT_MAX;	// 対応点間の距離の最大値( int 型 )
	int minCPDistanceInt = INT_MAX;		// 対応点間の距離の最小値( int 型 )
	int binNum;							// ヒストグラムのビンの数( 横軸 )
	int binMaxSize = -INT_MAX;			// ヒストグラムのビンの最大高( 縦軸 )


	/*** プログラム開始のコール ***/
	cout << "【 プログラム開始 】" << endl;


	/*** 対応点間の距離のエクセルデータの読み込み ***/
	cout << ">> 対応点間の距離データの読み込み中" << "\r";

	/* ファイル読み込み失敗時 */
	if( ( inputFile = fopen( CPDistanceFileName, "r" ) ) == NULL ){
		cout << "ファイル " << CPDistanceFileName << " が開けません" << endl;
		exit( 0 );
	}

	/* メモリの確保 */
	distances.resize( 0 );	// 対応点間の距離を格納する配列

	/* ファイル内容の読み込み */
	while( fscanf( inputFile, "%d,%f", &index, &distance ) != EOF ){

		/* 一定の小数位で切り落とした対応点間の距離値の算出 */
		distanceInt = (int)( distance * MULTIPLY_NUM );
		
		/* スタック配列に格納 */
		distances.push_back( distance );		// 対応点間の距離値
		distancesInt.push_back( distanceInt );	// 対応点間の距離値( int 型で MULTIPLY_NUM 倍後 )

		/* 対応点間の距離の和と二乗和の算出 */
		meanCPDistance += distance;
		rmse += distance * distance;

		/* 対応点間の距離の最大値と最小値の算出 */
		if( maxCPDistance < distance ) maxCPDistance = distance;
		if( distance < minCPDistance ) minCPDistance = distance;
		if( maxCPDistanceInt < distanceInt ) maxCPDistanceInt = distanceInt;
		if( distanceInt < minCPDistanceInt ) minCPDistanceInt = distanceInt;

	}
	
	/* メモリの解放 */
	fclose( inputFile );
	cout << ">> 対応点間の距離データの読み込み完了" << endl;
	cout << distancesInt.size() << endl;


	/*** 対応点間の距離の平均値と RMSE の算出 ***/
	meanCPDistance = meanCPDistance / distances.size();	// 平均値
	rmse = sqrt( rmse / distances.size() );				// RMSE


	/*** ヒストグラムの作成 ***/

	/* ヒストグラムのビン数の算出 */
	binNum = maxCPDistanceInt - minCPDistanceInt + 1;

	/* ビンのメモリ確保と初期化 */
	int *bin = new int[ binNum ];
	for( int i = 0; i < binNum; i++ ) bin[ i ] = 0;

	/* ヒストグラムを構成する各ビンの作成 */
	for( int i = 0; i < distancesInt.size(); i++ ){

		/* 対応点間の距離のカウント */
		bin[ distancesInt[ i ] - minCPDistanceInt ]++;
		
		//cout << "Index: " << i << ", Distance: " << distances[ i ] << endl;
		//cout << "Index: " << i << ", DistanceInt: " << distancesInt[ i ] << endl;
	}

	cout << "mean_CPDistance: " << meanCPDistance << endl;
	cout << "RMSE_CPDistance: " << rmse << endl;
	cout << "max_CPDistance: " << maxCPDistance << endl;
	cout << "min_CPDistance: " << minCPDistance << endl;
	cout << "max_CPDistance " << MULTIPLY_NUM << " 倍 : " << maxCPDistanceInt << endl;
	cout << "min_CPDistance " << MULTIPLY_NUM << " 倍 : " << minCPDistanceInt << endl;
	cout << "binNum: " << binNum << endl;
	
	/* ビンの高さの最大値の算出 */
	for( int i = 0; i < binNum; i++ ){

		if( binMaxSize < bin[ i ] ) binMaxSize = bin[ i ];

		/* 対応点間の距離とその出現回数のコマンドライン出力 */
		//cout << "BinNumber: " << i + 1 << ", Distance: " << minCPDistanceInt + i << ", Count: " << bin[ i ] << endl;
	}


	/*** ヒストグラムのビンのエクセルファイル出力 ***/

	/* ファイルストリーム変数の定義 */
	ofstream output;

	/* 出力ファイルパスの設定 */
	output.open( HistogramFileName );

	/* エクセル出力 */
	//for( int i = 0; i < binNum; i++ ) output << i << "," << bin[ i ] << endl;
	for( int i = 0; i < binNum; i++ ) output << minCPDistanceInt + i << "," << bin[ i ] << endl;

	/* メモリの解放 */
	output.close();


	/*** プログラム終了のコール ***/
	cout << "【 プログラム終了 】" << endl;


	/* メモリの解放 */
	delete bin;								// ヒストグラムのビン用配列


	return 0;
}