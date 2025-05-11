#ifndef __ParticleExporter_JSON_h__
#define __ParticleExporter_JSON_h__

#include "ExporterBase.h"
#include "SPlisHSPlasH/FluidModel.h"
#include <future>
#include <string>

namespace SPH
{
    /** \brief Exports particle snapshots as human‑readable JSON including
     *         particle attributes, an interpolated Eulerian vector field,
     *         and obstacle metadata.  See ParticleExporter_JSON.cpp.
     */
    class ParticleExporter_JSON : public ExporterBase
    {
    protected:
        std::string        m_exportPath;   //!< directory for .json output
        std::future<void>  m_handle;       //!< async file writer handle

        bool              m_gridInit = false;          //!< true after first init
        Vector3r          m_gridOrigin = Vector3r::Zero();
        Eigen::Vector3i   m_gridDims = Eigen::Vector3i::Ones();
        Real              m_gridDx = Real(0.0);      //!< == 0.5 * h

        /** \brief Serialize one frame.  Called internally by step().
         *  @param fileName  Absolute path without extension (".../file.json").
         *  @param model     Fluid model to export.
         *  @param objId     Optional object id when export‑splitting is enabled.
         */
        void writeParticlesJSON(const std::string &fileName,
                                FluidModel *model,
                                unsigned int frame,
                                unsigned int objId = 0xffffffff);

    public:
        ParticleExporter_JSON(SimulatorBase *base);
        ParticleExporter_JSON(const ParticleExporter_JSON &) = delete;
        ParticleExporter_JSON &operator=(const ParticleExporter_JSON &) = delete;
        ~ParticleExporter_JSON() override = default;

        // ExporterBase interface --------------------------------------------------
        virtual void init(const std::string &outputPath) override;
        virtual void step(unsigned int frame) override;
        virtual void reset() override;
        virtual void setActive(bool active) override;
    };
}

#endif // __ParticleExporter_JSON_h__
