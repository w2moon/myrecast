//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#include <cstdio>
#define _USE_MATH_DEFINES
#include <cmath>

#include "SDL.h"
#include "SDL_opengl.h"
#ifdef __APPLE__
#	include <OpenGL/glu.h>
#else
#	include <GL/glu.h>
#endif

#include <vector>
#include <string>

#include "imgui.h"
#include "imguiRenderGL.h"

#include "Recast.h"
#include "RecastDebugDraw.h"
#include "InputGeom.h"
#include "TestCase.h"
#include "Filelist.h"
#include "Sample_SoloMesh.h"
#include "Sample_TileMesh.h"
#include "Sample_TempObstacles.h"
#include "Sample_Debug.h"
#include "DetourNavMesh.h"
#include "DetourCommon.h"
#include "DetourTileCache.h"
#include "DetourTileCacheBuilder.h"
#include "DetourNavMeshQuery.h"
#include "DetourNavMeshBuilder.h"
#include "DetourCrowd.h"
#ifdef WIN32
#	define snprintf _snprintf
#	define putenv _putenv
#endif

using std::string;
using std::vector;

struct SampleItem
{
	Sample* (*create)();
	const string name;
};
Sample* createSolo() { return new Sample_SoloMesh(); }
Sample* createTile() { return new Sample_TileMesh(); }
Sample* createTempObstacle() { return new Sample_TempObstacles(); }
Sample* createDebug() { return new Sample_Debug(); }
static SampleItem g_samples[] =
{
	{ createSolo, "Solo Mesh" },
	{ createTile, "Tile Mesh" },
	{ createTempObstacle, "Temp Obstacles" },
};
static const int g_nsamples = sizeof(g_samples) / sizeof(SampleItem);
int doEditor();

////////////////////////////
// CUSTOM
//////////////
rcMeshLoaderObj* meshLoader;
static const int EXPECTED_LAYERS_PER_TILE = 4;

static const int MAX_POLYS = 25600;

static const int MAX_LAYERS = 3200;
int maxAgents = 1000;

////////////////////////////
// FROM SAMPLE
//////////////
InputGeom* m_geom;
dtNavMesh* m_navMesh;
dtNavMeshQuery* m_navQuery;
dtCrowd* m_crowd;

int m_maxTiles = 12800;
int m_maxPolysPerTile = 327680;
float m_tileSize = 48;

dtTileCache* m_tileCache;
int m_cacheCompressedSize;
int m_cacheRawSize;
int m_cacheLayerCount;
int m_cacheBuildMemUsage;
LinearAllocator* m_talloc;
FastLZCompressor* m_tcomp;
MeshProcess* m_tmproc;

unsigned char m_navMeshDrawFlags;

bool m_keepInterResults = false;
float m_totalBuildTimeMs;
float m_cacheBuildTimeMs;

int randModulo = 0;

//// DEFAULTS
float m_agentHeight = 2.0f;  // , 5.0f, 0.1f);
float m_agentRadius = 0.6f;  // , 5.0f, 0.1f);

float m_cellSize = 0.3f;
float m_cellHeight = 0.2f;

float m_agentMaxClimb = 0.9f;  // , 5.0f, 0.1f);
float m_agentMaxSlope = 30.0f;  // , 90.0f, 1.0f);

float m_regionMinSize = 8.0f;  // , 150.0f, 1.0f);
float m_regionMergeSize = 20.0f;  // , 150.0f, 1.0f);
bool m_monotonePartitioning = 1;

float m_edgeMaxLen = 12.0f;  // , 50.0f, 1.0f);
float m_edgeMaxError = 1.0f;  // , 3.0f, 0.1f);
float m_vertsPerPoly = 6.0f;  // , 12.0f, 1.0f);

float m_detailSampleDist = 6.0f;  // , 16.0f, 1.0f);
float m_detailSampleMaxError = 1.0f;  // , 16.0f, 1.0f);

unsigned char* m_triareas;
rcHeightfield* m_solid;
rcCompactHeightfield* m_chf;
rcContourSet* m_cset;
rcPolyMesh* m_pmesh;
rcConfig m_cfg;
rcPolyMeshDetail* m_dmesh;

static int calcLayerBufferSize(const int gridWidth, const int gridHeight)
{
    const int headerSize = dtAlign4(sizeof(dtTileCacheLayerHeader));
    const int gridSize = gridWidth * gridHeight;
    return headerSize + gridSize*4;
}
struct TileCacheData
{
    unsigned char* data;
    int dataSize;
};

struct MeshProcess : public dtTileCacheMeshProcess
{
    InputGeom* m_geom;
    
    inline MeshProcess() : m_geom(0)
    {
    }
    
    inline void init(InputGeom* geom)
    {
        m_geom = geom;
    }
    
    virtual void process(struct dtNavMeshCreateParams* params,
                         unsigned char* polyAreas, unsigned short* polyFlags)
    {
        // Update poly flags from areas.
        for (int i = 0; i < params->polyCount; ++i)
        {
            if (polyAreas[i] == DT_TILECACHE_WALKABLE_AREA) {
                // emscripten_log("set flag DT_TILECACHE_WALKABLE_AREA");
                polyAreas[i] = SAMPLE_POLYAREA_GROUND;
            }
            
            if (polyAreas[i] == SAMPLE_POLYAREA_GROUND ||
                polyAreas[i] == SAMPLE_POLYAREA_GRASS ||
                polyAreas[i] == SAMPLE_POLYAREA_ROAD)
            {
                // emscripten_log("set flag SAMPLE_POLYFLAGS_WALK");
                polyFlags[i] = SAMPLE_POLYFLAGS_WALK;
            }
            else if (polyAreas[i] == SAMPLE_POLYAREA_WATER)
            {
                // emscripten_log("set flag SAMPLE_POLYAREA_WATER");
                polyFlags[i] = SAMPLE_POLYFLAGS_SWIM;
            }
            else if (polyAreas[i] == SAMPLE_POLYAREA_DOOR)
            {
                // emscripten_log("set flag SAMPLE_POLYAREA_DOOR");
                polyFlags[i] = SAMPLE_POLYFLAGS_WALK | SAMPLE_POLYFLAGS_DOOR;
            }
        }
        
        // Pass in off-mesh connections.
        if (m_geom)
        {
            params->offMeshConVerts = m_geom->getOffMeshConnectionVerts();
            params->offMeshConRad = m_geom->getOffMeshConnectionRads();
            params->offMeshConDir = m_geom->getOffMeshConnectionDirs();
            params->offMeshConAreas = m_geom->getOffMeshConnectionAreas();
            params->offMeshConFlags = m_geom->getOffMeshConnectionFlags();
            params->offMeshConUserID = m_geom->getOffMeshConnectionId();
            params->offMeshConCount = m_geom->getOffMeshConnectionCount();
        }
    }
};
struct FastLZCompressor : public dtTileCacheCompressor
{
    virtual int maxCompressedSize(const int bufferSize)
    {
        return (int)(bufferSize* 1.05f);
    }
    
    virtual dtStatus compress(const unsigned char* buffer, const int bufferSize,
                              unsigned char* compressed, const int /*maxCompressedSize*/, int* compressedSize)
    {
        // *compressedSize = fastlz_compress((const void *const)buffer, bufferSize, compressed);
        memcpy(compressedSize, &bufferSize, sizeof(bufferSize));
        memcpy(compressed, buffer, bufferSize);
        
        return DT_SUCCESS;
    }
    
    virtual dtStatus decompress(const unsigned char* compressed, const int compressedSize,
                                unsigned char* buffer, const int maxBufferSize, int* bufferSize)
    {
        // *bufferSize = fastlz_decompress(compressed, compressedSize, buffer, maxBufferSize);
        memcpy(bufferSize, &compressedSize, sizeof(compressedSize));
        memcpy(buffer, compressed, compressedSize);
        
        return *bufferSize < 0 ? DT_FAILURE : DT_SUCCESS;
    }
};

struct LinearAllocator : public dtTileCacheAlloc
{
    unsigned char* buffer;
    int capacity;
    int top;
    int high;
    
    LinearAllocator(const int cap) : buffer(0), capacity(0), top(0), high(0)
    {
        resize(cap);
    }
    
    ~LinearAllocator()
    {
        dtFree(buffer);
    }
    
    void resize(const int cap)
    {
        if (buffer) dtFree(buffer);
        buffer = (unsigned char*)dtAlloc(cap, DT_ALLOC_PERM);
        capacity = cap;
    }
    
    virtual void reset()
    {
        high = dtMax(high, top);
        top = 0;
    }
    
    virtual void* alloc(const int size)
    {
        if (!buffer)
            return 0;
        if (top+size > capacity)
            return 0;
        unsigned char* mem = &buffer[top];
        top += size;
        return mem;
    }
    
    virtual void free(void* /*ptr*/)
    {
        // Empty
    }
};
struct RasterizationContext
{
    RasterizationContext() :
    solid(0),
    triareas(0),
    lset(0),
    chf(0),
    ntiles(0)
    {
        memset(tiles, 0, sizeof(TileCacheData)*MAX_LAYERS);
    }
    
    ~RasterizationContext()
    {
        rcFreeHeightField(solid);
        delete [] triareas;
        rcFreeHeightfieldLayerSet(lset);
        rcFreeCompactHeightfield(chf);
        for (int i = 0; i < MAX_LAYERS; ++i)
        {
            dtFree(tiles[i].data);
            tiles[i].data = 0;
        }
    }
    
    rcHeightfield* solid;
    unsigned char* triareas;
    rcHeightfieldLayerSet* lset;
    rcCompactHeightfield* chf;
    TileCacheData tiles[MAX_LAYERS];
    int ntiles;
};

static int rasterizeTileLayers(BuildContext* ctx, InputGeom* geom,
                               const int tx, const int ty,
                               const rcConfig& cfg,
                               TileCacheData* tiles,
                               const int maxTiles)
{
    if (!geom || !geom->getMesh() || !geom->getChunkyMesh())
    {
        return 0;
    }
    
    char buff[512];
    
    FastLZCompressor comp;
    RasterizationContext rc;
    
    const float* verts = geom->getMesh()->getVerts();
    const int nverts = geom->getMesh()->getVertCount();
    const rcChunkyTriMesh* chunkyMesh = geom->getChunkyMesh();
    
    // Tile bounds.
    const float tcs = cfg.tileSize * cfg.cs;
    
    rcConfig tcfg;
    memcpy(&tcfg, &cfg, sizeof(tcfg));
    
    tcfg.bmin[0] = cfg.bmin[0] + tx*tcs;
    tcfg.bmin[1] = cfg.bmin[1];
    tcfg.bmin[2] = cfg.bmin[2] + ty*tcs;
    tcfg.bmax[0] = cfg.bmin[0] + (tx+1)*tcs;
    tcfg.bmax[1] = cfg.bmax[1];
    tcfg.bmax[2] = cfg.bmin[2] + (ty+1)*tcs;
    tcfg.bmin[0] -= tcfg.borderSize*tcfg.cs;
    tcfg.bmin[2] -= tcfg.borderSize*tcfg.cs;
    tcfg.bmax[0] += tcfg.borderSize*tcfg.cs;
    tcfg.bmax[2] += tcfg.borderSize*tcfg.cs;
    
    // Allocate voxel heightfield where we rasterize our input data to.
    rc.solid = rcAllocHeightfield();
    if (!rc.solid)
    {
        return 0;
    }
    if (!rcCreateHeightfield(ctx, *rc.solid, tcfg.width, tcfg.height, tcfg.bmin, tcfg.bmax, tcfg.cs, tcfg.ch))
    {
        return 0;
    }
    
    // Allocate array that can hold triangle flags.
    // If you have multiple meshes you need to process, allocate
    // and array which can hold the max number of triangles you need to process.
    rc.triareas = new unsigned char[chunkyMesh->maxTrisPerChunk];
    if (!rc.triareas)
    {
        return 0;
    }
    
    float tbmin[2], tbmax[2];
    tbmin[0] = tcfg.bmin[0];
    tbmin[1] = tcfg.bmin[2];
    tbmax[0] = tcfg.bmax[0];
    tbmax[1] = tcfg.bmax[2];
    int cid[512];// TODO: Make grow when returning too many items.
    const int ncid = rcGetChunksOverlappingRect(chunkyMesh, tbmin, tbmax, cid, 512);
    if (!ncid)
    {
        // emscripten_log("no overlapping rect chunks");
        return 0; // empty
        
    } else {
        // sprintf(buff, "found %u overlapping rect chunks", ncid);
        // emscripten_log(buff);
    }
    
    for (int i = 0; i < ncid; ++i)
    {
        const rcChunkyTriMeshNode& node = chunkyMesh->nodes[cid[i]];
        const int* tris = &chunkyMesh->tris[node.i*3];
        const int ntris = node.n;
        
        memset(rc.triareas, 0, ntris*sizeof(unsigned char));
        rcMarkWalkableTriangles(ctx, tcfg.walkableSlopeAngle,
                                verts, nverts, tris, ntris, rc.triareas);
        
        rcRasterizeTriangles(ctx, verts, nverts, tris, rc.triareas, ntris, *rc.solid, tcfg.walkableClimb);
    }
    
    // Once all geometry is rasterized, we do initial pass of filtering to
    // remove unwanted overhangs caused by the conservative rasterization
    // as well as filter spans where the character cannot possibly stand.
    rcFilterLowHangingWalkableObstacles(ctx, tcfg.walkableClimb, *rc.solid);
    rcFilterLedgeSpans(ctx, tcfg.walkableHeight, tcfg.walkableClimb, *rc.solid);
    rcFilterWalkableLowHeightSpans(ctx, tcfg.walkableHeight, *rc.solid);
    
    
    rc.chf = rcAllocCompactHeightfield();
    if (!rc.chf)
    {
        return 0;
    }
    if (!rcBuildCompactHeightfield(ctx, tcfg.walkableHeight, tcfg.walkableClimb, *rc.solid, *rc.chf))
    {
        return 0;
    }
    
    // Erode the walkable area by agent radius.
    if (!rcErodeWalkableArea(ctx, tcfg.walkableRadius, *rc.chf))
    {
        return 0;
    }
    
    // (Optional) Mark areas.
    const ConvexVolume* vols = geom->getConvexVolumes();
    for (int i  = 0; i < geom->getConvexVolumeCount(); ++i)
    {
        // sprintf(buff, "MarkConvexPolyArea with %u vertices", vols[i].nverts);
        // emscripten_log(buff);
        
        rcMarkConvexPolyArea(ctx, vols[i].verts, vols[i].nverts,
                             vols[i].hmin, vols[i].hmax,
                             (unsigned char)vols[i].area, *rc.chf);
    }
    
    rc.lset = rcAllocHeightfieldLayerSet();
    if (!rc.lset)
    {
        return 0;
    }
    if (!rcBuildHeightfieldLayers(ctx, *rc.chf, tcfg.borderSize, tcfg.walkableHeight, *rc.lset))
    {
        return 0;
    }
    
    // sprintf(buff, "found %u layers", rc.lset->nlayers);
    // emscripten_log(buff);
    
    rc.ntiles = 0;
    for (int i = 0; i < rcMin(rc.lset->nlayers, MAX_LAYERS); ++i)
    {
        TileCacheData* tile = &rc.tiles[rc.ntiles++];
        const rcHeightfieldLayer* layer = &rc.lset->layers[i];
        
        // Store header
        dtTileCacheLayerHeader header;
        header.magic = DT_TILECACHE_MAGIC;
        header.version = DT_TILECACHE_VERSION;
        
        // Tile layer location in the navmesh.
        header.tx = tx;
        header.ty = ty;
        header.tlayer = i;
        dtVcopy(header.bmin, layer->bmin);
        dtVcopy(header.bmax, layer->bmax);
        
        // Tile info.
        header.width = (unsigned char)layer->width;
        header.height = (unsigned char)layer->height;
        header.minx = (unsigned char)layer->minx;
        header.maxx = (unsigned char)layer->maxx;
        header.miny = (unsigned char)layer->miny;
        header.maxy = (unsigned char)layer->maxy;
        header.hmin = (unsigned short)layer->hmin;
        header.hmax = (unsigned short)layer->hmax;
        
        dtStatus status = dtBuildTileCacheLayer(&comp, &header, layer->heights, layer->areas, layer->cons,
                                                &tile->data, &tile->dataSize);
        if (dtStatusFailed(status))
        {
            return 0;
        } else {
            // sprintf(buff, "Got an header of size %ux%u", header.width, header.height);
            // emscripten_log(buff);
        }
    }
    
    int n = 0;
    
    // Transfer ownsership of tile data from build context to the caller.
    for (int i = 0; i < rcMin(rc.ntiles, maxTiles); ++i)
    {
        tiles[n++] = rc.tiles[i];
        rc.tiles[i].data = 0;
        rc.tiles[i].dataSize = 0;
    }
    
    // sprintf(buff, "return %u tiles", n);
    // emscripten_log(buff);
    
    return n;
}
BuildContext* m_ctx;
float randZeroToOne()
{
    return 0.2;
}

void doTest(){
    BuildContext ctx;
    m_ctx = &ctx;
    m_geom = new InputGeom;
    m_geom->load(m_ctx, "Meshes/farm_01.obj");
    
    
    dtStatus status;
    char buff[512];
    
    if (!m_geom || !m_geom->getMesh())
    {
        return;
    }
    
    m_talloc = new LinearAllocator(64000);
    m_tcomp = new FastLZCompressor;
    m_tmproc = new MeshProcess;
    
    m_tmproc->init(m_geom);
    
    // Init cache
    const float* bmin = m_geom->getMeshBoundsMin();
    const float* bmax = m_geom->getMeshBoundsMax();
    int gw = 0, gh = 0;
    rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);
    const int ts = (int)m_tileSize;
    const int tw = (gw + ts-1) / ts;
    const int th = (gh + ts-1) / ts;
    
    int tileBits = rcMin((int)dtIlog2(dtNextPow2(tw*th*EXPECTED_LAYERS_PER_TILE)), 14);
    if (tileBits > 14) tileBits = 14;
    int polyBits = 22 - tileBits;
    m_maxTiles = 1 << tileBits;
    m_maxPolysPerTile = 1 << polyBits;
    
    sprintf(buff, "bmin=%f  bmax=%f  gw=%u  gh=%u  ts=%u  tw=%u  th=%u  m_maxTiles=%u  m_maxPolysPerTile=%u  offMeshCons=%u", *bmin, *bmax, gw, gh, ts, tw, th, m_maxTiles, m_maxPolysPerTile, m_geom->getOffMeshConnectionCount());
   
    
    // Generation params.
    rcConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.cs = m_cellSize;
    cfg.ch = m_cellHeight;
    cfg.walkableSlopeAngle = m_agentMaxSlope;
    cfg.walkableHeight = (int)ceilf(m_agentHeight / cfg.ch);
    cfg.walkableClimb = (int)floorf(m_agentMaxClimb / cfg.ch);
    cfg.walkableRadius = (int)ceilf(m_agentRadius / cfg.cs);
    cfg.maxEdgeLen = (int)(m_edgeMaxLen / m_cellSize);
    cfg.maxSimplificationError = m_edgeMaxError;
    cfg.minRegionArea = (int)rcSqr(m_regionMinSize);        // Note: area = size*size
    cfg.mergeRegionArea = (int)rcSqr(m_regionMergeSize);    // Note: area = size*size
    cfg.maxVertsPerPoly = (int)m_vertsPerPoly;
    cfg.tileSize = (int)m_tileSize;
    cfg.borderSize = cfg.walkableRadius + 3; // Reserve enough padding.
    cfg.width = cfg.tileSize + cfg.borderSize*2;
    cfg.height = cfg.tileSize + cfg.borderSize*2;
    cfg.detailSampleDist = m_detailSampleDist < 0.9f ? 0 : m_cellSize * m_detailSampleDist;
    cfg.detailSampleMaxError = m_cellHeight * m_detailSampleMaxError;
    rcVcopy(cfg.bmin, bmin);
    rcVcopy(cfg.bmax, bmax);
    
    // Tile cache params.
    dtTileCacheParams tcparams;
    memset(&tcparams, 0, sizeof(tcparams));
    rcVcopy(tcparams.orig, bmin);
    tcparams.cs = m_cellSize;
    tcparams.ch = m_cellHeight;
    tcparams.width = (int)m_tileSize;
    tcparams.height = (int)m_tileSize;
    tcparams.walkableHeight = m_agentHeight;
    tcparams.walkableRadius = m_agentRadius;
    tcparams.walkableClimb = m_agentMaxClimb;
    tcparams.maxSimplificationError = m_edgeMaxError;
    tcparams.maxTiles = tw*th*EXPECTED_LAYERS_PER_TILE;
    tcparams.maxObstacles = 1280;
    
    dtFreeTileCache(m_tileCache);
    
    m_tileCache = dtAllocTileCache();
    if (!m_tileCache)
    {
        return ;
    }
    status = m_tileCache->init(&tcparams, m_talloc, m_tcomp, m_tmproc);
    if (dtStatusFailed(status))
    {
        return ;
    }
    
    dtFreeNavMesh(m_navMesh);
    
    m_navMesh = dtAllocNavMesh();
    if (!m_navMesh)
    {
        return ;
    }
    
    dtNavMeshParams params;
    memset(&params, 0, sizeof(params));
    rcVcopy(params.orig, m_geom->getMeshBoundsMin());
    params.tileWidth = m_tileSize*m_cellSize;
    params.tileHeight = m_tileSize*m_cellSize;
    params.maxTiles = m_maxTiles;
    params.maxPolys = m_maxPolysPerTile;
    
    // sprintf(buff, "initNavMesh  tileWidth=%f  tileHeight=%f  maxTiles=%u  maxPolys=%u", params.tileWidth, params.tileHeight, params.maxTiles, params.maxPolys);
    // emscripten_log(buff);
    
    status = m_navMesh->init(&params);
    if (dtStatusFailed(status))
    {
        return;
    }
    
    m_navQuery = dtAllocNavMeshQuery();
    if (!m_navQuery)
    {
        dtFree(m_navQuery);
        return ;
    }
    
    status = m_navQuery->init(m_navMesh, 20480);
    if (dtStatusFailed(status))
    {
        return ;
    }
    
    // Preprocess tiles.
    
    m_cacheLayerCount = 0;
    m_cacheCompressedSize = 0;
    m_cacheRawSize = 0;
    
    for (int y = 0; y < th; ++y)
    {
        for (int x = 0; x < tw; ++x)
        {
            TileCacheData tiles[MAX_LAYERS];
            memset(tiles, 0, sizeof(tiles));
            int ntiles = rasterizeTileLayers(m_ctx, m_geom, x, y, cfg, tiles, MAX_LAYERS);
            
            // sprintf(buff, "found %i rasterized tiles", ntiles);
            // emscripten_log(buff);
            
            for (int i = 0; i < ntiles; ++i)
            {
                TileCacheData* tile = &tiles[i];
                
                // sprintf(buff, "tile data: %u", sizeof(tile->data));
                // emscripten_log(buff);
                
                status = m_tileCache->addTile(tile->data, tile->dataSize, DT_COMPRESSEDTILE_FREE_DATA, 0);
                if (dtStatusFailed(status))
                {
                    dtFree(tile->data);
                    tile->data = 0;
                    continue;
                }
                
                m_cacheLayerCount++;
                m_cacheCompressedSize += tile->dataSize;
                m_cacheRawSize += calcLayerBufferSize(tcparams.width, tcparams.height);
            }
        }
    }
    
    sprintf(buff, "Build initial %u tiles", th*tw);
    // Build initial meshes
    for (int y = 0; y < th; ++y) {
        for (int x = 0; x < tw; ++x) {
            // sprintf(buff, "buildNavMeshTilesAt(%u, %u)", x, y);
            // emscripten_log(buff);
            status = m_tileCache->buildNavMeshTilesAt(x, y, m_navMesh);
            if (dtStatusFailed(status))
            {
                sprintf(buff, "Failed to buildNavMeshTilesAt %ux%u", x, y);
            }
        }
    }
    
    // emscripten_log("Build initial meshes done");
    
    m_cacheBuildMemUsage = m_talloc->high;
    
    const dtNavMesh* nav = m_navMesh;
    int navmeshMemUsage = 0;
    for (int i = 0; i < nav->getMaxTiles(); ++i)
    {
        const dtMeshTile* tile = nav->getTile(i);
        if (tile->header) {
            navmeshMemUsage += tile->dataSize;
        }
    }
    
    dtQueryFilter filter;
    filter.setIncludeFlags(3);
    filter.setExcludeFlags(0);
    
    dtPolyRef ref = 0;
    
    float randomPt[3];
    
    status = m_navQuery->findRandomPoint(&filter, randZeroToOne, &ref, randomPt);
    
    if (dtStatusFailed(status)) {
        printf("Cannot find a random point: %u\n", status);
        
    } else {
        
    }
    
   }

void doSave(int argc, char** argv){
    if(argc < 2){
        printf("need objfilePath and outfilePath\n");
        return;
    }
    char* objFilePath = argv[1];
    char* outfilePath = argv[2];
    
    InputGeom* geom = new InputGeom;
    Sample* sample = 0;
    
    BuildContext ctx;
    if(!geom->load(&ctx, objFilePath)){
        printf("not found objFile :%s\n",objFilePath);
        return;
    }
    
    sample = g_samples[2].create();
    
        sample->setContext(&ctx);
        if (geom)
        {
            sample->handleMeshChanged(geom);
        }
        else{
            printf("not valid geom :%s\n",objFilePath);
            return;
        }
    
       if (sample && geom)
    {
        sample->handleMeshChanged(geom);
    }
       else{
           printf("not valid sample :%s\n",objFilePath);
           return;
       }
    
    // This will ensure that tile & poly bits are updated in tiled sample.
    if (sample){
         sample->handleSettings();
    }
    else{
        printf("not valid settings :%s\n",objFilePath);
        return;
    }
    
    
    if (sample && sample->handleBuild())
    {
        ((Sample_TempObstacles*)sample)->saveAll(outfilePath);
    }
    else{
        printf("not valid build :%s\n",objFilePath);
        return;
    }
    

    
    
}

int main(int argc, char** argv)
{
    
    //doTest();
   //doEditor();
    doSave(argc,argv);
	
	return 0;
}

int doEditor(){
    // Init SDL
    if (SDL_Init(SDL_INIT_EVERYTHING) != 0)
    {
        printf("Could not initialise SDL.\nError: %s\n", SDL_GetError());
        return -1;
    }
    
    // Enable depth buffer.
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    
    // Set color channel depth.
    SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);
    
    // 4x MSAA.
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);
    
    SDL_DisplayMode displayMode;
    SDL_GetCurrentDisplayMode(0, &displayMode);
    
    bool presentationMode = false;
    Uint32 flags = SDL_WINDOW_OPENGL;
    int width;
    int height;
    if (presentationMode)
    {
        // Create a fullscreen window at the native resolution.
        width = displayMode.w;
        height = displayMode.h;
        flags |= SDL_WINDOW_FULLSCREEN;
    }
    else
    {
        float aspect = 16.0f / 9.0f;
        width = rcMin(displayMode.w, (int)(displayMode.h * aspect)) - 80;
        height = displayMode.h - 80;
    }
    
    SDL_Window* window;
    SDL_Renderer* renderer;
    int errorCode = SDL_CreateWindowAndRenderer(width, height, flags, &window, &renderer);
    
    if (errorCode != 0 || !window || !renderer)
    {
        printf("Could not initialise SDL opengl\nError: %s\n", SDL_GetError());
        return -1;
    }
    
    SDL_SetWindowPosition(window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
    SDL_GL_CreateContext(window);
    
    if (!imguiRenderGLInit("DroidSans.ttf"))
    {
        printf("Could not init GUI renderer.\n");
        SDL_Quit();
        return -1;
    }
    
    float t = 0.0f;
    float timeAcc = 0.0f;
    Uint32 prevFrameTime = SDL_GetTicks();
    int mousePos[2] = {0, 0};
    int origMousePos[2] = {0, 0}; // Used to compute mouse movement totals across frames.
    
    float cameraEulers[] = {45, -45};
    float cameraPos[] = {0, 0, 0};
    float camr = 1000;
    float origCameraEulers[] = {0, 0}; // Used to compute rotational changes across frames.
    
    float moveW = 0, moveS = 0, moveA = 0, moveD = 0;
    
    float scrollZoom = 0;
    bool rotate = false;
    bool movedDuringRotate = false;
    float rayStart[3];
    float rayEnd[3];
    bool mouseOverMenu = false;
    
    bool showMenu = !presentationMode;
    bool showLog = false;
    bool showTools = true;
    bool showLevels = false;
    bool showSample = false;
    bool showTestCases = false;
    
    // Window scroll positions.
    int propScroll = 0;
    int logScroll = 0;
    int toolsScroll = 0;
    
    string sampleName = "Choose Sample...";
    
    vector<string> files;
    const string meshesFolder = "Meshes";
    string meshName = "Choose Mesh...";
    
    float markerPosition[3] = {0, 0, 0};
    bool markerPositionSet = false;
    
    InputGeom* geom = 0;
    Sample* sample = 0;
    
    const string testCasesFolder = "TestCases";
    TestCase* test = 0;
    
    BuildContext ctx;
    
    // Fog.
    float fogColor[4] = { 0.32f, 0.31f, 0.30f, 1.0f };
    glEnable(GL_FOG);
    glFogi(GL_FOG_MODE, GL_LINEAR);
    glFogf(GL_FOG_START, camr * 0.1f);
    glFogf(GL_FOG_END, camr * 1.25f);
    glFogfv(GL_FOG_COLOR, fogColor);
    
    glEnable(GL_CULL_FACE);
    glDepthFunc(GL_LEQUAL);
    
    bool done = false;
    while(!done)
    {
        // Handle input events.
        int mouseScroll = 0;
        bool processHitTest = false;
        bool processHitTestShift = false;
        SDL_Event event;
        
        while (SDL_PollEvent(&event))
        {
            switch (event.type)
            {
                case SDL_KEYDOWN:
                    // Handle any key presses here.
                    if (event.key.keysym.sym == SDLK_ESCAPE)
                    {
                        done = true;
                    }
                    else if (event.key.keysym.sym == SDLK_t)
                    {
                        showLevels = false;
                        showSample = false;
                        showTestCases = true;
                        scanDirectory(testCasesFolder, ".txt", files);
                    }
                    else if (event.key.keysym.sym == SDLK_TAB)
                    {
                        showMenu = !showMenu;
                    }
                    else if (event.key.keysym.sym == SDLK_SPACE)
                    {
                        if (sample)
                            sample->handleToggle();
                    }
                    else if (event.key.keysym.sym == SDLK_1)
                    {
                        if (sample)
                            sample->handleStep();
                    }
                    else if (event.key.keysym.sym == SDLK_9)
                    {
                        if (sample && geom)
                        {
                            string savePath = meshesFolder + "/";
                            BuildSettings settings;
                            memset(&settings, 0, sizeof(settings));
                            
                            rcVcopy(settings.navMeshBMin, geom->getNavMeshBoundsMin());
                            rcVcopy(settings.navMeshBMax, geom->getNavMeshBoundsMax());
                            
                            sample->collectSettings(settings);
                            
                            geom->saveGeomSet(&settings);
                        }
                    }
                    break;
                    
                case SDL_MOUSEWHEEL:
                    if (event.wheel.y < 0)
                    {
                        // wheel down
                        if (mouseOverMenu)
                        {
                            mouseScroll++;
                        }
                        else
                        {
                            scrollZoom += 1.0f;
                        }
                    }
                    else
                    {
                        if (mouseOverMenu)
                        {
                            mouseScroll--;
                        }
                        else
                        {
                            scrollZoom -= 1.0f;
                        }
                    }
                    break;
                case SDL_MOUSEBUTTONDOWN:
                    if (event.button.button == SDL_BUTTON_RIGHT)
                    {
                        if (!mouseOverMenu)
                        {
                            // Rotate view
                            rotate = true;
                            movedDuringRotate = false;
                            origMousePos[0] = mousePos[0];
                            origMousePos[1] = mousePos[1];
                            origCameraEulers[0] = cameraEulers[0];
                            origCameraEulers[1] = cameraEulers[1];
                        }
                    }
                    break;
                    
                case SDL_MOUSEBUTTONUP:
                    // Handle mouse clicks here.
                    if (event.button.button == SDL_BUTTON_RIGHT)
                    {
                        rotate = false;
                        if (!mouseOverMenu)
                        {
                            if (!movedDuringRotate)
                            {
                                processHitTest = true;
                                processHitTestShift = true;
                            }
                        }
                    }
                    else if (event.button.button == SDL_BUTTON_LEFT)
                    {
                        if (!mouseOverMenu)
                        {
                            processHitTest = true;
                            processHitTestShift = (SDL_GetModState() & KMOD_SHIFT) ? true : false;
                        }
                    }
                    
                    break;
                    
                case SDL_MOUSEMOTION:
                    mousePos[0] = event.motion.x;
                    mousePos[1] = height-1 - event.motion.y;
                    
                    if (rotate)
                    {
                        int dx = mousePos[0] - origMousePos[0];
                        int dy = mousePos[1] - origMousePos[1];
                        cameraEulers[0] = origCameraEulers[0] - dy * 0.25f;
                        cameraEulers[1] = origCameraEulers[1] + dx * 0.25f;
                        if (dx * dx + dy * dy > 3 * 3)
                        {
                            movedDuringRotate = true;
                        }
                    }
                    break;
                    
                case SDL_QUIT:
                    done = true;
                    break;
                    
                default:
                    break;
            }
        }
        
        unsigned char mouseButtonMask = 0;
        if (SDL_GetMouseState(0, 0) & SDL_BUTTON_LMASK)
            mouseButtonMask |= IMGUI_MBUT_LEFT;
        if (SDL_GetMouseState(0, 0) & SDL_BUTTON_RMASK)
            mouseButtonMask |= IMGUI_MBUT_RIGHT;
        
        Uint32 time = SDL_GetTicks();
        float dt = (time - prevFrameTime) / 1000.0f;
        prevFrameTime = time;
        
        t += dt;
        
        // Hit test mesh.
        if (processHitTest && geom && sample)
        {
            float hitTime;
            bool hit = geom->raycastMesh(rayStart, rayEnd, hitTime);
            bool out = false;
            if(!hit){
                hit = true;
                out = true;
            }
            
            if (hit)
            {
                if (SDL_GetModState() & KMOD_CTRL)
                {
                    // Marker
                    markerPositionSet = true;
                    markerPosition[0] = rayStart[0] + (rayEnd[0] - rayStart[0]) * hitTime;
                    markerPosition[1] = rayStart[1] + (rayEnd[1] - rayStart[1]) * hitTime;
                    markerPosition[2] = rayStart[2] + (rayEnd[2] - rayStart[2]) * hitTime;
                }
                else
                {
                    float pos[3];
                    pos[0] = rayStart[0] + (rayEnd[0] - rayStart[0]) * hitTime;
                    pos[1] = rayStart[1] + (rayEnd[1] - rayStart[1]) * hitTime;
                    pos[2] = rayStart[2] + (rayEnd[2] - rayStart[2]) * hitTime;
                    printf("%f %f %f\n",pos[0],pos[1],pos[2]);
                    if(out){
                       // pos[2] = 200;
                    }
                    sample->handleClick(rayStart, pos, processHitTestShift);
                }
            }
            else
            {
                if (SDL_GetModState() & KMOD_CTRL)
                {
                    // Marker
                    markerPositionSet = false;
                }
            }
        }
        
        // Update sample simulation.
        const float SIM_RATE = 20;
        const float DELTA_TIME = 1.0f / SIM_RATE;
        timeAcc = rcClamp(timeAcc + dt, -1.0f, 1.0f);
        int simIter = 0;
        while (timeAcc > DELTA_TIME)
        {
            timeAcc -= DELTA_TIME;
            if (simIter < 5 && sample)
            {
                sample->handleUpdate(DELTA_TIME);
            }
            simIter++;
        }
        
        // Clamp the framerate so that we do not hog all the CPU.
        const float MIN_FRAME_TIME = 1.0f / 40.0f;
        if (dt < MIN_FRAME_TIME)
        {
            int ms = (int)((MIN_FRAME_TIME - dt) * 1000.0f);
            if (ms > 10) ms = 10;
            if (ms >= 0) SDL_Delay(ms);
        }
        
        // Set the viewport.
        glViewport(0, 0, width, height);
        GLint viewport[4];
        glGetIntegerv(GL_VIEWPORT, viewport);
        
        // Clear the screen
        glClearColor(0.3f, 0.3f, 0.32f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glDisable(GL_TEXTURE_2D);
        glEnable(GL_DEPTH_TEST);
        
        // Compute the projection matrix.
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(50.0f, (float)width/(float)height, 1.0f, camr);
        GLdouble projectionMatrix[16];
        glGetDoublev(GL_PROJECTION_MATRIX, projectionMatrix);
        
        // Compute the modelview matrix.
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glRotatef(cameraEulers[0], 1, 0, 0);
        glRotatef(cameraEulers[1], 0, 1, 0);
        glTranslatef(-cameraPos[0], -cameraPos[1], -cameraPos[2]);
        GLdouble modelviewMatrix[16];
        glGetDoublev(GL_MODELVIEW_MATRIX, modelviewMatrix);
        
        // Get hit ray position and direction.
        GLdouble x, y, z;
        gluUnProject(mousePos[0], mousePos[1], 0.0f, modelviewMatrix, projectionMatrix, viewport, &x, &y, &z);
        rayStart[0] = (float)x;
        rayStart[1] = (float)y;
        rayStart[2] = (float)z;
        gluUnProject(mousePos[0], mousePos[1], 1.0f, modelviewMatrix, projectionMatrix, viewport, &x, &y, &z);
        rayEnd[0] = (float)x;
        rayEnd[1] = (float)y;
        rayEnd[2] = (float)z;
        
        // Handle keyboard movement.
        const Uint8* keystate = SDL_GetKeyboardState(NULL);
        moveW = rcClamp(moveW + dt * 4 * (keystate[SDL_SCANCODE_W] ? 1 : -1), 0.0f, 1.0f);
        moveA = rcClamp(moveA + dt * 4 * (keystate[SDL_SCANCODE_A] ? 1 : -1), 0.0f, 1.0f);
        moveS = rcClamp(moveS + dt * 4 * (keystate[SDL_SCANCODE_S] ? 1 : -1), 0.0f, 1.0f);
        moveD = rcClamp(moveD + dt * 4 * (keystate[SDL_SCANCODE_D] ? 1 : -1), 0.0f, 1.0f);
        
        float keybSpeed = 22.0f;
        if (SDL_GetModState() & KMOD_SHIFT)
        {
            keybSpeed *= 4.0f;
        }
        
        float movex = (moveD - moveA) * keybSpeed * dt;
        float movey = (moveS - moveW) * keybSpeed * dt + scrollZoom * 2.0f;
        scrollZoom = 0;
        
        cameraPos[0] += movex * (float)modelviewMatrix[0];
        cameraPos[1] += movex * (float)modelviewMatrix[4];
        cameraPos[2] += movex * (float)modelviewMatrix[8];
        
        cameraPos[0] += movey * (float)modelviewMatrix[2];
        cameraPos[1] += movey * (float)modelviewMatrix[6];
        cameraPos[2] += movey * (float)modelviewMatrix[10];
        
        glEnable(GL_FOG);
        
        if (sample)
            sample->handleRender();
        if (test)
            test->handleRender();
        
        glDisable(GL_FOG);
        
        // Render GUI
        glDisable(GL_DEPTH_TEST);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluOrtho2D(0, width, 0, height);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        
        mouseOverMenu = false;
        
        imguiBeginFrame(mousePos[0], mousePos[1], mouseButtonMask, mouseScroll);
        
        if (sample)
        {
            sample->handleRenderOverlay((double*)projectionMatrix, (double*)modelviewMatrix, (int*)viewport);
        }
        if (test)
        {
            if (test->handleRenderOverlay((double*)projectionMatrix, (double*)modelviewMatrix, (int*)viewport))
                mouseOverMenu = true;
        }
        
        // Help text.
        if (showMenu)
        {
            const char msg[] = "W/S/A/D: Move  RMB: Rotate";
            imguiDrawText(280, height-20, IMGUI_ALIGN_LEFT, msg, imguiRGBA(255,255,255,128));
        }
        
        if (showMenu)
        {
            if (imguiBeginScrollArea("Properties", width-250-10, 10, 250, height-20, &propScroll))
                mouseOverMenu = true;
            
            if (imguiCheck("Show Log", showLog))
                showLog = !showLog;
            if (imguiCheck("Show Tools", showTools))
                showTools = !showTools;
            
            imguiSeparator();
            imguiLabel("Sample");
            if (imguiButton(sampleName.c_str()))
            {
                if (showSample)
                {
                    showSample = false;
                }
                else
                {
                    showSample = true;
                    showLevels = false;
                    showTestCases = false;
                }
            }
            
            imguiSeparator();
            imguiLabel("Input Mesh");
            if (imguiButton(meshName.c_str()))
            {
                if (showLevels)
                {
                    showLevels = false;
                }
                else
                {
                    showSample = false;
                    showTestCases = false;
                    showLevels = true;
                    scanDirectory(meshesFolder, ".obj", files);
                    scanDirectoryAppend(meshesFolder, ".gset", files);
                }
            }
            if (geom)
            {
                char text[64];
                snprintf(text, 64, "Verts: %.1fk  Tris: %.1fk",
                         geom->getMesh()->getVertCount()/1000.0f,
                         geom->getMesh()->getTriCount()/1000.0f);
                imguiValue(text);
            }
            imguiSeparator();
            
            if (geom && sample)
            {
                imguiSeparatorLine();
                
                sample->handleSettings();
                
                if (imguiButton("Build"))
                {
                    ctx.resetLog();
                    if (!sample->handleBuild())
                    {
                        showLog = true;
                        logScroll = 0;
                    }
                    ctx.dumpLog("Build log %s:", meshName.c_str());
                    
                    // Clear test.
                    delete test;
                    test = 0;
                }
                
                imguiSeparator();
            }
            
            if (sample)
            {
                imguiSeparatorLine();
                sample->handleDebugMode();
            }
            
            imguiEndScrollArea();
        }
        
        // Sample selection dialog.
        if (showSample)
        {
            static int levelScroll = 0;
            if (imguiBeginScrollArea("Choose Sample", width-10-250-10-200, height-10-250, 200, 250, &levelScroll))
                mouseOverMenu = true;
            
            Sample* newSample = 0;
            for (int i = 0; i < g_nsamples; ++i)
            {
                if (imguiItem(g_samples[i].name.c_str()))
                {
                    newSample = g_samples[i].create();
                    if (newSample)
                        sampleName = g_samples[i].name;
                }
            }
            if (newSample)
            {
                delete sample;
                sample = newSample;
                sample->setContext(&ctx);
                if (geom)
                {
                    sample->handleMeshChanged(geom);
                }
                showSample = false;
            }
            
            if (geom || sample)
            {
                const float* bmin = 0;
                const float* bmax = 0;
                if (geom)
                {
                    bmin = geom->getNavMeshBoundsMin();
                    bmax = geom->getNavMeshBoundsMax();
                }
                // Reset camera and fog to match the mesh bounds.
                if (bmin && bmax)
                {
                    camr = sqrtf(rcSqr(bmax[0]-bmin[0]) +
                                 rcSqr(bmax[1]-bmin[1]) +
                                 rcSqr(bmax[2]-bmin[2])) / 2;
                    cameraPos[0] = (bmax[0] + bmin[0]) / 2 + camr;
                    cameraPos[1] = (bmax[1] + bmin[1]) / 2 + camr;
                    cameraPos[2] = (bmax[2] + bmin[2]) / 2 + camr;
                    camr *= 3;
                }
                cameraEulers[0] = 45;
                cameraEulers[1] = -45;
                glFogf(GL_FOG_START, camr*0.1f);
                glFogf(GL_FOG_END, camr*1.25f);
            }
            
            imguiEndScrollArea();
        }
        
        // Level selection dialog.
        if (showLevels)
        {
            static int levelScroll = 0;
            if (imguiBeginScrollArea("Choose Level", width - 10 - 250 - 10 - 200, height - 10 - 450, 200, 450, &levelScroll))
                mouseOverMenu = true;
            
            vector<string>::const_iterator fileIter = files.begin();
            vector<string>::const_iterator filesEnd = files.end();
            vector<string>::const_iterator levelToLoad = filesEnd;
            for (; fileIter != filesEnd; ++fileIter)
            {
                if (imguiItem(fileIter->c_str()))
                {
                    levelToLoad = fileIter;
                }
            }
            
            if (levelToLoad != filesEnd)
            {
                meshName = *levelToLoad;
                showLevels = false;
                
                delete geom;
                geom = 0;
                
                string path = meshesFolder + "/" + meshName;
                
                geom = new InputGeom;
                if (!geom->load(&ctx, path))
                {
                    delete geom;
                    geom = 0;
                    
                    // Destroy the sample if it already had geometry loaded, as we've just deleted it!
                    if (sample && sample->getInputGeom())
                    {
                        delete sample;
                        sample = 0;
                    }
                    
                    showLog = true;
                    logScroll = 0;
                    ctx.dumpLog("Geom load log %s:", meshName.c_str());
                }
                if (sample && geom)
                {
                    sample->handleMeshChanged(geom);
                }
                
                if (geom || sample)
                {
                    const float* bmin = 0;
                    const float* bmax = 0;
                    if (geom)
                    {
                        bmin = geom->getNavMeshBoundsMin();
                        bmax = geom->getNavMeshBoundsMax();
                    }
                    // Reset camera and fog to match the mesh bounds.
                    if (bmin && bmax)
                    {
                        camr = sqrtf(rcSqr(bmax[0]-bmin[0]) +
                                     rcSqr(bmax[1]-bmin[1]) +
                                     rcSqr(bmax[2]-bmin[2])) / 2;
                        cameraPos[0] = (bmax[0] + bmin[0]) / 2 + camr;
                        cameraPos[1] = (bmax[1] + bmin[1]) / 2 + camr;
                        cameraPos[2] = (bmax[2] + bmin[2]) / 2 + camr;
                        camr *= 3;
                    }
                    cameraEulers[0] = 45;
                    cameraEulers[1] = -45;
                    glFogf(GL_FOG_START, camr * 0.1f);
                    glFogf(GL_FOG_END, camr * 1.25f);
                }
            }
            
            imguiEndScrollArea();
            
        }
        
        // Test cases
        if (showTestCases)
        {
            static int testScroll = 0;
            if (imguiBeginScrollArea("Choose Test To Run", width-10-250-10-200, height-10-450, 200, 450, &testScroll))
                mouseOverMenu = true;
            
            vector<string>::const_iterator fileIter = files.begin();
            vector<string>::const_iterator filesEnd = files.end();
            vector<string>::const_iterator testToLoad = filesEnd;
            for (; fileIter != filesEnd; ++fileIter)
            {
                if (imguiItem(fileIter->c_str()))
                {
                    testToLoad = fileIter;
                }
            }
            
            if (testToLoad != filesEnd)
            {
                string path = testCasesFolder + "/" + *testToLoad;
                test = new TestCase;
                if (test)
                {
                    // Load the test.
                    if (!test->load(path))
                    {
                        delete test;
                        test = 0;
                    }
                    
                    // Create sample
                    Sample* newSample = 0;
                    for (int i = 0; i < g_nsamples; ++i)
                    {
                        if (g_samples[i].name == test->getSampleName())
                        {
                            newSample = g_samples[i].create();
                            if (newSample)
                                sampleName = g_samples[i].name;
                        }
                    }
                    
                    delete sample;
                    sample = newSample;
                    
                    if (sample)
                    {
                        sample->setContext(&ctx);
                        showSample = false;
                    }
                    
                    // Load geom.
                    meshName = test->getGeomFileName();
                    
                    
                    path = meshesFolder + "/" + meshName;
                    
                    delete geom;
                    geom = new InputGeom;
                    if (!geom || !geom->load(&ctx, path))
                    {
                        delete geom;
                        geom = 0;
                        delete sample;
                        sample = 0;
                        showLog = true;
                        logScroll = 0;
                        ctx.dumpLog("Geom load log %s:", meshName.c_str());
                    }
                    if (sample && geom)
                    {
                        sample->handleMeshChanged(geom);
                    }
                    
                    // This will ensure that tile & poly bits are updated in tiled sample.
                    if (sample)
                        sample->handleSettings();
                    
                    ctx.resetLog();
                    if (sample && !sample->handleBuild())
                    {
                        ctx.dumpLog("Build log %s:", meshName.c_str());
                    }
                    
                    if (geom || sample)
                    {
                        const float* bmin = 0;
                        const float* bmax = 0;
                        if (geom)
                        {
                            bmin = geom->getNavMeshBoundsMin();
                            bmax = geom->getNavMeshBoundsMax();
                        }
                        // Reset camera and fog to match the mesh bounds.
                        if (bmin && bmax)
                        {
                            camr = sqrtf(rcSqr(bmax[0] - bmin[0]) +
                                         rcSqr(bmax[1] - bmin[1]) +
                                         rcSqr(bmax[2] - bmin[2])) / 2;
                            cameraPos[0] = (bmax[0] + bmin[0]) / 2 + camr;
                            cameraPos[1] = (bmax[1] + bmin[1]) / 2 + camr;
                            cameraPos[2] = (bmax[2] + bmin[2]) / 2 + camr;
                            camr *= 3;
                        }
                        cameraEulers[0] = 45;
                        cameraEulers[1] = -45;
                        glFogf(GL_FOG_START, camr * 0.2f);
                        glFogf(GL_FOG_END, camr * 1.25f);
                    }
                    
                    // Do the tests.
                    if (sample)
                        test->doTests(sample->getNavMesh(), sample->getNavMeshQuery());
                }
            }				
            
            imguiEndScrollArea();
        }
        
        
        // Log
        if (showLog && showMenu)
        {
            if (imguiBeginScrollArea("Log", 250 + 20, 10, width - 300 - 250, 200, &logScroll))
                mouseOverMenu = true;
            for (int i = 0; i < ctx.getLogCount(); ++i)
                imguiLabel(ctx.getLogText(i));
            imguiEndScrollArea();
        }
        
        // Left column tools menu
        if (!showTestCases && showTools && showMenu) // && geom && sample)
        {
            if (imguiBeginScrollArea("Tools", 10, 10, 250, height - 20, &toolsScroll))
                mouseOverMenu = true;
            
            if (sample)
                sample->handleTools();
            
            imguiEndScrollArea();
        }
        
        // Marker
        if (markerPositionSet && gluProject((GLdouble)markerPosition[0], (GLdouble)markerPosition[1], (GLdouble)markerPosition[2],
                                            modelviewMatrix, projectionMatrix, viewport, &x, &y, &z))
        {
            // Draw marker circle
            glLineWidth(5.0f);
            glColor4ub(240,220,0,196);
            glBegin(GL_LINE_LOOP);
            const float r = 25.0f;
            for (int i = 0; i < 20; ++i)
            {
                const float a = (float)i / 20.0f * RC_PI*2;
                const float fx = (float)x + cosf(a)*r;
                const float fy = (float)y + sinf(a)*r;
                glVertex2f(fx,fy);
            }
            glEnd();
            glLineWidth(1.0f);
        }
        
        imguiEndFrame();
        imguiRenderGLDraw();		
        
        glEnable(GL_DEPTH_TEST);
        SDL_GL_SwapWindow(window);
    }
    
    imguiRenderGLDestroy();
    
    SDL_Quit();
    
    delete sample;
    delete geom;
    return 0;
}
