/***** PCD データの PLY データへの変換 *****/

//【 コマンドライン引数 】
// 1. 入力点群ファイル名
// ⇒ PCD データ
// 2. 出力点群ファイル名
// ⇒ PLY データ
// 3. 点群ファイルタイプ
// ⇒ 1:PCD( PointXYZRGB ), 2:PCD( PointXYZ )

// 例：
// Output/EdgePointData/ID_0002_LiverV31U10L1R10.pcd Output/EdgePointData/ID_0002_LiverV31U10L1R10.ply 1


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
	cout << "----- Convert PCDData to PLYData Start -----" << endl;


	/* コマンドライン引数読み込み失敗時 */
	if( argc < 3 ){
		cerr << "Caution! : コマンドライン引数エラー" << endl;
		cerr << "Usage: " << endl;
		cerr << argv[ 0 ] << endl << "1. inputFileName" << endl
			<< "2. outputFileName" << endl
			<< "3. KindOfPointCloud ⇒ 1:PointXYZRGB, 2:PointXYZ" << endl;
		return EXIT_FAILURE;
	}


	/* 変数の宣言 */
	const char * inputFilename = argv[ 1 ];		// 入力ファイル名
	const char * outputFilename = argv[ 2 ];	// 出力ファイル名
	int KindOfPointCloud;						// 点群ファイルタイプ ⇒ 1:PCD( PointXYZRGB ), 2:PCD( PointXYZ )

	/* メモリの確保 */
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr inputCloud1( new pcl::PointCloud< pcl::PointXYZRGB > );	// PointXYZRGB 型を指定
	pcl::PointCloud< pcl::PointXYZ >::Ptr inputCloud2( new pcl::PointCloud< pcl::PointXYZ > );			// PointXYZ 型を指定

	/* 各種プロパティの取得 */
	KindOfPointCloud = atoi( argv[ 3 ] );	// 点群ファイルタイプ ⇒ 1:PCD( PointXYZRGB ), 2:PCD( PointXYZ )
	

	/*** 点群の入力 ***/
	if( KindOfPointCloud == 1 ) loadPCDPointCloudRGB( inputCloud1, inputFilename );	// 入力点群が PointXYZRGB の場合
	if( KindOfPointCloud == 2 ) loadPCDPointCloud( inputCloud2, inputFilename );	// 入力点群が PointXYZ の場合


	/*** 点群数のコマンドライン出力 ***/
	if( KindOfPointCloud == 1 ) cout << "点群数: " << inputCloud1->size() << " 個" << endl;
	if( KindOfPointCloud == 2 ) cout << "点群数: " << inputCloud2->size() << " 個" << endl;


	/*** 点群の出力 ***/
	if( KindOfPointCloud == 1 ) savePointCloudRGBtoPLY( inputCloud1, outputFilename );	// 出力点群が PointXYZRGB の場合
	if( KindOfPointCloud == 2 ) savePointCloudtoPLY( inputCloud2, outputFilename );	// 出力点群が PointXYZ の場合


	/*** プログラム終了のコール ***/
	cout << "----- Convert PCDData to PLYData End -----" << endl;


	return 0;
}