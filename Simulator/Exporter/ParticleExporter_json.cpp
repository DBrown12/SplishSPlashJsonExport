// ParticleExporter_JSON.cpp  – JSON exporter with vector field and obstacle data
// Requires: nlohmann/json single‑header (json.hpp)  

#include <fstream>
#include <future>
#include <map>

#include "Simulator/Exporter/ParticleExporter_JSON.h"
#include "SPlisHSPlasH/Simulation.h"
#include "SPlisHSPlasH/TimeManager.h"
#include "Utilities/FileSystem.h"
#include "Utilities/Logger.h"
#include "SPlisHSPlasH/BoundaryModel.h"
#include "SPlisHSPlasH/RigidBodyObject.h"
#include "Simulator/Exporter/ParticleExporter_JSON.h"


using json = nlohmann::json;
using namespace Utilities;
//using namespace SPH;

/*class ParticleExporter_JSON : public ExporterBase*/
using namespace SPH;


ParticleExporter_JSON::ParticleExporter_JSON(SimulatorBase* base)
    : ExporterBase(base) 
{

}


void ParticleExporter_JSON::init(const std::string &outputPath)
{
    m_exportPath = FileSystem::normalizePath(outputPath + "/json");
}

/*
void ParticleExporter_JSON::step(unsigned int frame)
{
    if (!m_active) return;
    Simulation *sim = Simulation::getCurrent();
    for (unsigned int i = 0; i < sim->numberOfFluidModels(); ++i)
    {
        FluidModel *model = sim->getFluidModel(i);
        const unsigned int numActiveParticles = model->numActiveParticles();
        //going to go through all my active particles and get their information and keep
        //track of them.
        for (unsigned int j = 0; j < numActiveParticles; ++j) {
                
            unsigned int particleID = model->getParticleId(j);
            std::string name = "ParticleData_" + model->getId() + "_particle_" + std::to_string(particleID) + "_frame_" + std::to_string(frame);
            //unsigned int objID = 0xffffffff;
            //unsigned int particleID = model->getParticleId(j);
            writeParticlesJSON(m_exportPath + "/" + name + ".json", model, j, frame);
        }
    }
}*/

void ParticleExporter_JSON::reset() {}

void ParticleExporter_JSON::setActive(bool active)
{
    ExporterBase::setActive(active);
    if (m_active) FileSystem::makeDirs(m_exportPath);
}
/*
private:
std::string m_exportPath;
std::future<void> m_handle;
*/
// Cubic poly6 kernel (scalar)
static inline Real W_poly6(const Real r, const Real h)
{
    const Real q = r / h;
    if (q >= Real(2.0)) return Real(0.0);
    const Real coef = Real(315.0) / (Real(64.0) * M_PI * pow(h, 9));
    if (q < Real(1.0))
        return coef * (pow(2.0 - q, 3) - Real(4.0) * pow(1.0 - q, 3));
    return coef * pow(2.0 - q, 3);
}


void ParticleExporter_JSON::step(unsigned int frame)
{
    if (!m_active) return;
    Simulation* sim = Simulation::getCurrent();
    for (unsigned int mi = 0; mi < sim->numberOfFluidModels(); ++mi)
    {
        FluidModel* model = sim->getFluidModel(mi);
        std::string name = "ParticleData_" + model->getId() + "_" +
            std::to_string(frame);
        writeParticlesJSON(m_exportPath + "/" + name + ".json",
            model,
            frame,
            0xffffffff);
    }
}

    

void ParticleExporter_JSON::writeParticlesJSON(const std::string& fileName,
    FluidModel* model,
    unsigned int      frame,
    unsigned int      objId)
{
    Simulation *sim = Simulation::getCurrent();
    json root;
    root["frame"] = frame;
    root["dt"]    = TimeManager::getCurrent()->getTimeStepSize();

    // ------------- Serialize particles -------------------------------------------------
    auto &plist = root["particles"];
        
    Utilities::SceneLoader* sceneLoader = m_base->getSceneLoader();

    std::string particleAttributes = "velocity";

    sceneLoader->readValue<std::string>("Configuration", "particleAttributes", particleAttributes);

    std::vector<std::string> attributes;
    Utilities::StringTools::tokenize(particleAttributes, attributes, ";");
    //StringTools::tokenize(m_base->getValue<std::string>(SimulatorBase::PARTICLE_EXPORT_ATTRIBUTES), attributes, ";");

    std::map<unsigned int, int> attrMap; // token idx -> field idx
    for (unsigned int ai = 0; ai < attributes.size(); ++ai)
    {
        if (attributes[ai] == "position" || attributes[ai] == "velocity") { attrMap[ai] = -1; continue; }
        attrMap[ai] = -1;
        for (unsigned int fi = 0; fi < model->numberOfFields(); ++fi)
            if (model->getField(fi).name == attributes[ai]) { attrMap[ai] = fi; break; }
    }

    const unsigned int n = model->numActiveParticles();
    for (unsigned int pi = 0; pi < n; ++pi)
    {
        if (objId != 0xffffffff && model->getObjectId(pi) != objId) continue;

        json p;
        p["id"]  = model->getParticleId(pi);
        const Vector3r &x = model->getPosition(pi);
        const Vector3r &v = model->getVelocity(pi);
        p["pos"] = { x[0], x[1], x[2] };
        p["vel"] = { v[0], v[1], v[2] };

        json attr;
        for (unsigned int ai = 0; ai < attributes.size(); ++ai)
        {
            int fi = attrMap[ai];
            if (fi == -1) continue;
            const FieldDescription &field = model->getField(fi);
            if (field.type == Scalar)
                attr[field.name] = *((Real*)field.getFct(pi));
            else if (field.type == UInt)
                attr[field.name] = *((unsigned int*)field.getFct(pi));
            else if (field.type == Vector3)
            {
                Eigen::Map<const Vector3r> vv((Real*)field.getFct(pi));
                attr[field.name] = { vv[0], vv[1], vv[2] };
            }
        }
        if (!attr.empty()) p["attr"] = std::move(attr);
        plist.push_back(std::move(p));
    }

    // ------------- Build velocity vector field on grid ------------------------------
    //const Real h  = model->getSupportRadius();
    const Real h = Simulation::getCurrent()->getSupportRadius();

    const Real dx = Real(0.5) * h;
    Vector3r minB, maxB;
    //sim->computeBounds(minB, maxB);
    minB = model->getPosition(0), maxB = minB;
    for (unsigned int i = 1; i < model->numActiveParticles(); ++i)
        minB = minB.cwiseMin(model->getPosition(i)),
        maxB = maxB.cwiseMax(model->getPosition(i));
        
    Eigen::Vector3i dims = ((maxB - minB) / dx).cast<int>();
    for (int c = 0; c < 3; ++c) dims[c] = std::max(dims[c], 1);

    std::vector<Vector3r>   vel(dims.x()*dims.y()*dims.z(), Vector3r::Zero());
    std::vector<Real>       w  (dims.x()*dims.y()*dims.z(), Real(0));

    auto flat = [&](const Eigen::Vector3i &ijk){ return (ijk[2]*dims.y() + ijk[1])*dims.x() + ijk[0]; };

    for (unsigned int pi = 0; pi < n; ++pi)
    {
        if (objId != 0xffffffff && model->getObjectId(pi) != objId) continue;
        const Vector3r &xp = model->getPosition(pi);
        const Vector3r &vp = model->getVelocity(pi);
        Eigen::Vector3i minI = ((xp - Vector3r::Constant(h) - minB) / dx).cast<int>();
        Eigen::Vector3i maxI = ((xp + Vector3r::Constant(h) - minB) / dx).cast<int>();
        minI = minI.cwiseMax(Eigen::Vector3i::Zero());
        maxI = maxI.cwiseMin(dims - Eigen::Vector3i::Ones());
        for (int z = minI.z(); z <= maxI.z(); ++z)
        for (int y = minI.y(); y <= maxI.y(); ++y)
        for (int x = minI.x(); x <= maxI.x(); ++x)
        {
            Eigen::Vector3i ijk(x,y,z);
            Vector3r xg = minB + (ijk.cast<Real>() + Vector3r::Constant(0.5))*dx;
            Real r = (xg - xp).norm(); if (r >= Real(2.0)*h) continue;
            Real weight = W_poly6(r,h);
            size_t idx = flat(ijk);
            vel[idx] += vp * weight;
            w[idx]   += weight;
        }
    }
    for (size_t k = 0; k < vel.size(); ++k) if (w[k] > Real(0)) vel[k] /= w[k];

    // Serialize vector field as nested array [z][y][x][3]
    json vf;
    vf["name"]   = "velocity";
    vf["shape"]  = { dims.z(), dims.y(), dims.x() };
    vf["dx"]     = dx;
    vf["origin"] = { minB[0], minB[1], minB[2] };
    auto &nested = vf["dataNested"];
    nested = json::array();
    size_t idx = 0;
    for (int z = 0; z < dims.z(); ++z)
    {
        json sliceY = json::array();
        for (int y = 0; y < dims.y(); ++y)
        {
            json rowX = json::array();
            for (int x = 0; x < dims.x(); ++x)
            {
                const Vector3r &vcell = vel[idx++];
                rowX.push_back({ vcell[0], vcell[1], vcell[2] });
            }
            sliceY.push_back(std::move(rowX));
        }
        nested.push_back(std::move(sliceY));
    }
    root["vectorField"] = std::move(vf);

    // ------------- Obstacles ---------------------------------------------------------
    auto &obsArr = root["obstacles"];
    for (unsigned int bi = 0; bi < sim->numberOfBoundaryModels(); ++bi)
    {
        BoundaryModel *bm = sim->getBoundaryModel(bi);
        RigidBodyObject *rb = bm->getRigidBodyObject();
        json ob;

        ob["id"] = reinterpret_cast<std::uintptr_t>(rb);
        const Vector3r& vobs = rb->getVelocity();

        ob["velocity"] = { vobs[0], vobs[1],vobs[2] };

        static unsigned nextID = 0;
        std::size_t ptrHash = reinterpret_cast<std::size_t>(rb);
        unsigned id = static_cast<unsigned>(ptrHash & 0xffffffff);

        //------------ going to build our own AABB ---------
        const auto& verts = rb->getVertices();
        Vector3r bmin = verts.front();
        Vector3r bmax = bmin;
        for (const auto& v : verts)
        {
            bmin = bmin.cwiseMin(v);
            bmax = bmax.cwiseMax(v);
        }

        ob["bounds"] = { {"min", {bmin[0], bmin[1], bmin[2]}}, {"max", {bmax[0], bmax[1], bmax[2]}} };


        /*
        ob["id"] = rb->getId();
        ob["type"] = "mesh"; // or "box/sphere" depending on your own RTTI
        const Vector3r &vobs = rb->getVelocity();
        ob["velocity"] = { vobs[0], vobs[1], vobs[2] };
        // local AABB in world space
        const Vector3r &bmin = rb->getAABB().min();
        const Vector3r &bmax = rb->getAABB().max();
        ob["bounds"] = { {"min", {bmin[0], bmin[1], bmin[2]}}, {"max", {bmax[0], bmax[1], bmax[2]}} };
        obsArr.push_back(std::move(ob));
        */
    }

    // ------------- Async write --------------------------------------------------------
    const bool async = m_base->getValue<bool>(SimulatorBase::ASYNC_EXPORT);
    if (async)
    {
        if (m_handle.valid()) m_handle.wait();
        m_handle = std::async(std::launch::async, [root, fileName](){ std::ofstream out(fileName); out << root.dump(2); });
    }
    else
    {
        std::ofstream out(fileName);
        out << root.dump(2);
    }
}

