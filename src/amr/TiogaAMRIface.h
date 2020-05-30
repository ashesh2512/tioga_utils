#ifndef TIOGAAMRIFACE_H
#define TIOGAAMRIFACE_H

#include <memory>

#include "StructMesh.h"

namespace YAML {
class Node;
}

namespace TIOGA {
class tioga;
}

namespace tioga_amr {

class TiogaAMRIface
{
public:
    TiogaAMRIface();

    ~TiogaAMRIface();

    void load(const YAML::Node&);

    void initialize();

    void register_mesh(TIOGA::tioga&);

private:
    std::unique_ptr<StructMesh> m_mesh;

    //! Reference to cell variable field
    Field* m_qcell{nullptr};

    //! Reference to node variable field
    Field* m_qnode{nullptr};

    //! Number of ghost cells
    int m_num_ghost{3};

    //! Number of components in the cell field
    int m_ncell_vars{0};

    //! Number of components for the node field
    int m_nnode_vars{0};

    //! Integers per grid for TIOGA call
    static constexpr int ints_per_grid{10};

    //! Reals per grid for TIOGA call
    static constexpr int reals_per_grid{6};
};

}

#endif /* TIOGAAMRIFACE_H */