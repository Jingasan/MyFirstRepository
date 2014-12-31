/***** 設定ヘッダファイル *****/


/*** フラグの管理 ***/
#define COMMENT	// コメントのコマンドライン出力 ON：出力, OFF：非出力
#define MERGE	// 点群数正規化後のモデル点群の統合 ON：統合, OFF：非統合
#define VIEWER	// 点群の表示 ON：表示, OFF：非表示


/*** 定数の定義 ***/

/* 入力するモデル点群ファイル数 */
#define INPUTPOINTCLOUDNUM 30

/* モデル点群ファイルの数 */
#define POINTCLOUDNUM 30

/* 入力点群ファイル名 */
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

/* 出力点群ファイル名 */
const char OutputPLYFileName1[] = "Output/PrincipalComponentAnalysis/merged30PointCloud.ply";