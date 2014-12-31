/***** 3次元点群数の削減 *****/

//【 コマンドライン引数 】
// 1. 入力点群ファイル名
// ⇒ PCD データ
// 2. 出力点群ファイル名
// ⇒ PCD データ
// 3. 点群ファイルタイプ
// ⇒ 1:PCD( PointXYZRGB ), 2:PCD( PointXYZ )
// 4. ボクセルグリッドサイズ
// ⇒ float 型 [単位:m]

// 例：
// Output/EdgePointData/ID_0002_LiverV31U10L1.pcd Output/EdgePointData/ID_0002_LiverV31U10L1R10.pcd 1 10.0


/*** インクルードファイル ***/

/* PCL 1.6.0 */
#include <pcl/point_types.h>		// 点群のデータ型

/* 作成ヘッダファイル */
#include "FunctionDefinition.h"	// 関数の定義


/*** 名前空間の宣言 ***/
using namespace std;


/*** メイン処理文 ***/
int main( int argc, char* argv[] ){


	/*** プログラム開始のコール ***/
	cout << "----- VoxelGrid による点群数の削減開始 -----" << endl;


	/* コマンドライン引数読み込み失敗時 */
	if( argc < 5 ){
		cerr << "Caution! : コマンドライン引数エラー" << endl;
		cerr << "Usage: " << endl;
		cerr << argv[ 0 ] << endl << "1. inputFileName"
			<< endl << "2. outputFileName"
			<< endl << "3. KindOfPointCloud ⇒ 1:PCD 2:PLY"
			<< endl << "[ 4. VoxelGridSize ]" << endl;
		return EXIT_FAILURE;
	}


	/* 変数の宣言 */
	const char * inputFilename = argv[ 1 ];		// 入力ファイル名
	const char * outputFilename = argv[ 2 ];	// 出力ファイル名
	int KindOfPointCloud;						// 点群ファイルタイプ ⇒ 1:PCD( PointXYZRGB ), 2:PCD( PointXYZ ), 3:PLY( PointXYZRGB ), 4:PLY( PointXYZ )
	float VoxelGridSize;						// ボクセルグリッドサイズ[単位:m]
	float boxelGridSizeX = 10.00f;				// 既定ボクセルグリッドサイズ X
	float boxelGridSizeY = 10.00f;				// 既定ボクセルグリッドサイズ Y
	float boxelGridSizeZ = 10.00f;				// 既定ボクセルグリッドサイズ Z

	/* メモリの確保 */
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr inputCloud1( new pcl::PointCloud< pcl::PointXYZRGB > );	// PointXYZRGB 型を指定
	pcl::PointCloud< pcl::PointXYZ >::Ptr inputCloud2( new pcl::PointCloud< pcl::PointXYZ > );			// PointXYZ 型を指定
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr outputCloud1( new pcl::PointCloud< pcl::PointXYZRGB > );	// PointXYZRGB 型を指定
	pcl::PointCloud< pcl::PointXYZ >::Ptr outputCloud2( new pcl::PointCloud< pcl::PointXYZ > );			// PointXYZ 型を指定

	/* 各種プロパティの取得 */
	KindOfPointCloud = atoi( argv[ 3 ] );	// 点群ファイルタイプ ⇒ 1:PCD( PointXYZRGB ), 2:PCD( PointXYZ ), 3:PLY( PointXYZRGB ), 4:PLY( PointXYZ )
	if( argc > 3 ){
		VoxelGridSize = atof( argv[ 4 ] );	// ボクセルグリッドサイズ[単位:m]
		boxelGridSizeX = VoxelGridSize;		// ボクセルグリッドサイズ X
		boxelGridSizeY = VoxelGridSize;		// ボクセルグリッドサイズ Y
		boxelGridSizeZ = VoxelGridSize;		// ボクセルグリッドサイズ Z
	}
	

	/*** 点群の入力 ***/
	if( KindOfPointCloud == 1 ) loadPCDPointCloudRGB( inputCloud1, inputFilename );	// 入力点群が PCD かつ PointXYZRGB の場合
	if( KindOfPointCloud == 2 ) loadPCDPointCloud( inputCloud2, inputFilename );	// 入力点群が PCD かつ PointXYZ の場合
	if( KindOfPointCloud == 3 ) loadPLYPointCloudRGB( inputCloud1, inputFilename );	// 入力点群が PLY かつ PointXYZRGB の場合
	if( KindOfPointCloud == 4 ) loadPLYPointCloud( inputCloud2, inputFilename );	// 入力点群が PLY かつ PointXYZ の場合


	/*** 点群数のコマンドライン出力 ***/
	if( KindOfPointCloud == 1 || KindOfPointCloud == 3 ) cout << "削減前点群数: " << inputCloud1->size() << " 個" << endl;
	if( KindOfPointCloud == 2 || KindOfPointCloud == 4 ) cout << "削減前点群数: " << inputCloud2->size() << " 個" << endl;


	/*** 点群数の削減 ***/
	if( KindOfPointCloud == 1 || KindOfPointCloud == 3 ) reductPointCloudRGB( inputCloud1, outputCloud1, boxelGridSizeX, boxelGridSizeY, boxelGridSizeZ );
	if( KindOfPointCloud == 2 || KindOfPointCloud == 4 ) reductPointCloud( inputCloud2, outputCloud2, boxelGridSizeX, boxelGridSizeY, boxelGridSizeZ );


	/*** 点群数のコマンドライン出力 ***/
	if( KindOfPointCloud == 1 || KindOfPointCloud == 3 ) cout << "削減後点群数: " << outputCloud1->size() << " 個" << endl;
	if( KindOfPointCloud == 2 || KindOfPointCloud == 4 ) cout << "削減後点群数: " << outputCloud2->size() << " 個" << endl;


	/*** 点群の出力 ***/
	if( KindOfPointCloud == 1 ) savePointCloudRGBtoPCD( outputCloud1, outputFilename );
	if( KindOfPointCloud == 2 ) savePointCloudtoPCD( outputCloud2, outputFilename );
	if( KindOfPointCloud == 3 ) savePointCloudRGBtoPLY( outputCloud1, outputFilename );;
	if( KindOfPointCloud == 4 ) savePointCloudtoPLY( outputCloud2, outputFilename );


	/*** プログラム終了のコール ***/
	cout << "----- VoxelGrid による点群数の削減終了 -----" << endl;


	return 0;
}