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
#include "SPlisHsPlasH/Simulation.h"


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


void ParticleExporter_JSON::reset() {}

void ParticleExporter_JSON::setActive(bool active)
{
    ExporterBase::setActive(active);
    if (m_active) FileSystem::makeDirs(m_exportPath);
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

    const int kernelID = sim->getKernel();
    const Real currentSimSupRadius = Simulation::getCurrent()->getSupportRadius();

    using KernelFct = std::function<Real(Real)>;
    KernelFct W;
    Real W_zero = Real(0);
    std::string kernelName;

    //I can't actually call the kernel directly it seems, so making a helper function
    //So that I can actually get the correct kernel.
    switch(kernelID) 
    {
    
    case 0: //Cubickernel
        CubicKernel::setRadius(currentSimSupRadius);
        W = [=](Real r) {return CubicKernel::W(r); };
        W_zero = CubicKernel::W_zero();
        kernelName = "CubicKernel";
        break;


    case 1: //WendlandQuintiC2Kernel
        WendlandQuinticC2Kernel::setRadius(currentSimSupRadius);
        W = [=](Real r) {return WendlandQuinticC2Kernel::W(r); };
        W_zero = WendlandQuinticC2Kernel::W_zero();
        kernelName = "WendlandQuintiC2Kernel";
        break;

    case 2: // poly6Kernel
        Poly6Kernel::setRadius(currentSimSupRadius);
        W = [=](Real r) {return Poly6Kernel::W(r); };
        W_zero = Poly6Kernel::W_zero();
        kernelName = "poly6Kernel";
        break;

    case 3:
        SpikyKernel::setRadius(currentSimSupRadius);
        W = [=](Real r) {return SpikyKernel::W(r); };
        W_zero = SpikyKernel::W_zero();
        kernelName = "SpikyKernel";
        break;

    case 4: //Pre-computer cubic kernel
    default:
        Simulation::PrecomputedCubicKernel::setRadius(currentSimSupRadius);
        W = [=](Real r) { return Simulation::PrecomputedCubicKernel::W(r); };
        W_zero = Simulation::PrecomputedCubicKernel::W_zero();
        kernelName = "PrecomputedCubicKernel";
        break;
        
    }

    root["kernel"] = {

        {"Kernel id", kernelID},
        {"Kernel Name", kernelName}

    };


    if (!m_gridInit)
    {
        m_gridDx = Real(0.5) * currentSimSupRadius;

        // bounds of the *initial* fluid configuration
        Vector3r minB = model->getPosition(0);
        Vector3r maxB = minB;
        for (unsigned int pi = 1; pi < model->numActiveParticles(); ++pi)
        {
            minB = minB.cwiseMin(model->getPosition(pi));
            maxB = maxB.cwiseMax(model->getPosition(pi));
        }
        m_gridOrigin = minB;
        m_gridDims = ((maxB - minB) / m_gridDx).cast<int>();
        for (int c = 0; c < 3; ++c) m_gridDims[c] = std::max(m_gridDims[c], 1);

        m_gridInit = true;
    }
    //const Real W0 = W_poly6(Real(0), h);
    const Real   dx = m_gridDx;
    const Vector3r& minB = m_gridOrigin;
    const Eigen::Vector3i dims = m_gridDims;

    //  Kernel chosen by the simulator
// ---------------------------------------------------------------------
    //const auto  W = sim->getKernel();   // function pointer: Real (*)(Real, Real)
    //const Real  W_zero = sim->W_zero();      // kernel self‑weight
    const Real  h = sim->getSupportRadius();

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
            if (model->getField(fi).name == attributes[ai]) { 
                attrMap[ai] = fi; break; 
            }
    }

    //Seralizing my particles + rasterising my velocity field.
    const unsigned int n = model->numActiveParticles();

    //Gonna test this
    std::vector<Vector3r> vel(dims.x() * dims.y() * dims.z(), Vector3r::Zero());
    std::vector<Real> w(dims.x() * dims.y() * dims.z(), Real(0));
    auto flat = [&](const Eigen::Vector3i& ijk) {
        return (ijk[2]*dims.y() + ijk[1])*dims.x() + ijk[0]; };
    
    for (unsigned int pi = 0; pi < n; ++pi)
    {
        if (objId != 0xffffffff && model->getObjectId(pi) != objId) continue;

        json p;
        json attr;

        const Vector3r &x = model->getPosition(pi);
        const Vector3r &v = model->getVelocity(pi);
        const Real mass = model->getMass(pi);
        const Real density = model->getDensity(pi);
        const Real mash_over_density = mass / density;

        p["id"] = model->getParticleId(pi);
        p["pos"] = { x[0], x[1], x[2] };
        p["vel"] = { v[0], v[1], v[2] };
        p["mass"] = mass;
        p["density"] = density;
        p["W0"] = W_zero;

        for (unsigned int ai = 0; ai < attributes.size(); ++ai)
        {
            int fi = attrMap[ai];
            if (fi == -1) continue;
            const FieldDescription &field = model->getField(fi);
            if (field.type == Scalar)
                attr[field.name] = *reinterpret_cast<const Real*>(field.getFct(pi));
            else if (field.type == UInt)
                attr[field.name] = *reinterpret_cast<const unsigned int*>(field.getFct(pi));
            else if (field.type == Vector3)
            {
                Eigen::Map<const Vector3r> vv(reinterpret_cast<const Real*>(field.getFct(pi)));
                attr[field.name] = { vv[0], vv[1], vv[2] };
            }
        }
        if (!attr.empty()) p["attr"] = std::move(attr);
        plist.push_back(std::move(p));

    // ------------- Build velocity vector field on grid ------------------------------
    //const Real h  = model->getSupportRadius();

    //sim->computeBounds(minB, maxB);
    /*
    minB = model->getPosition(0), maxB = minB;
    for (unsigned int i = 1; i < model->numActiveParticles(); ++i)
        minB = minB.cwiseMin(model->getPosition(i)),
        maxB = maxB.cwiseMax(model->getPosition(i));
    */

    //rastorising
        
        Eigen::Vector3i minI = ((x - Vector3r::Constant(h) - minB) / dx).cast<int>();
        Eigen::Vector3i maxI = ((x + Vector3r::Constant(h) - minB) / dx).cast<int>();
        minI = minI.cwiseMax(Eigen::Vector3i::Zero());
        maxI = maxI.cwiseMin(dims - Eigen::Vector3i::Ones());

        for (int z = minI.z(); z <= maxI.z(); ++z)
            for (int y = minI.y(); y <= maxI.y(); ++y)
                for (int xg = minI.x(); xg <= maxI.x(); ++xg)
                {
                    Eigen::Vector3i ijk(xg, y, z);
                    Vector3r xgWorld = minB + (ijk.cast<Real>() + Vector3r::Constant(0.5)) * dx;
                    Real r = (xgWorld - x).norm();
                    if (r >= Real(2.0) * h) continue;

                    Real weight = mash_over_density * W(r);
                    size_t idx = flat(ijk);
                    vel[idx] += v * weight;
                    w[idx] += weight;
                }
    }
    
    for (size_t k = 0; k < vel.size(); ++k) if (w[k] > Real(0)) vel[k] /= w[k];
  

    // Serialize vector field as nested array [z][y][x][3]
    json vf;
    vf["name"]   = "velocity";
    vf["shape"]  = { dims.z(), dims.y(), dims.x() };
    vf["dx"]     = dx;
    //We want the vector field to be over the 
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
    //Simulation* sim = Simulation::getCurrent();
    for (unsigned int bi = 0; bi < sim->numberOfBoundaryModels(); ++bi)
    {
        BoundaryModel *bm = sim->getBoundaryModel(bi);
        RigidBodyObject *rb = bm->getRigidBodyObject();
        json ob;

        ob["Obstacle id"] = reinterpret_cast<std::uintptr_t>(rb);
        const Vector3r& vobs = rb->getVelocity();

        ob["Obstacle velocity"] = { vobs[0], vobs[1],vobs[2] };

        ob["mass"] = rb->getMass();

        //------------ going to build our own AABB ---------
        const auto& verts = rb->getVertices();
        Vector3r bmin = verts.front();
        Vector3r bmax = bmin;
        for (const auto &v : verts)
        {
            bmin = bmin.cwiseMin(v);
            bmax = bmax.cwiseMax(v);
        }

        ob["bounds"] = { {"min", {bmin[0], bmin[1], bmin[2]}}, {"max", {bmax[0], bmax[1], bmax[2]}} };

        obsArr.push_back(std::move(ob));

    }
    // --- Also export minimal JSON with only particle positions for splashsurf ---
    {
        std::vector<std::array<Real, 3>> positionsOnly;
        for (unsigned int pi = 0; pi < n; ++pi)
        {
            if (objId != 0xffffffff && model->getObjectId(pi) != objId)
                continue;

            const Vector3r& x = model->getPosition(pi);
            positionsOnly.push_back({ x[0], x[1], x[2] });
        }

        // Write to a parallel file with _positions.json suffix
        std::string posFileName = fileName;
        const size_t ext = posFileName.rfind(".json");
        if (ext != std::string::npos)
            posFileName.replace(ext, 5, "_positions.json");
        else
            posFileName += "_positions.json";

        const bool async = m_base->getValue<bool>(SimulatorBase::ASYNC_EXPORT);
        if (async)
        {
            // Avoid race conditions with shared m_handle
            std::async(std::launch::async, [positionsOnly, posFileName]() {
                std::ofstream out(posFileName);
                out << json(positionsOnly).dump(2);
                });
        }
        else
        {
            std::ofstream out(posFileName);
            out << json(positionsOnly).dump(2);
        }
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

