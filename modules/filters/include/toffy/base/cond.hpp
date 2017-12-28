#pragma once
/*
 * @file nop.hpp
 *
 * Copyright (c) 2015
 * VoXel Interaction Design GmbH
 *
 * VoXel.at <office@voxel.at>
 * @author Simon Vogl <simon@voxel.at>
 * @date 4 Mar 2016
 * @version 1.0
 *
 * @brief
 *
 * A nop filter - doing nothing.
 *
 * Rationale: use the <nop> </nop> elements to inactivate portions of a filter bank, no need for xml comments that cannot be nested.
 *
 * Usage:
 */

#include "toffy/filterbank.hpp"

namespace toffy {
namespace filters {
/**
 * @brief A nop filter - doing nothing.
 * @ingroup Filters
 *
 * Rationale: use the \<nop> \</nop> elements to inactivate portions of a filter
 * bank, no need for xml comments that cannot be nested.
 */
class Cond : public FilterBank {

public:
    Cond();
    virtual ~Cond() {}


    //virtual int loadConfig(const boost::property_tree::ptree& pt);

    virtual boost::property_tree::ptree getConfig() const;
    //virtual void updateConfig(const boost::property_tree::ptree &pt);
    virtual bool filter(const Frame& in, Frame& out);

private:
    bool enabled; ///< switch if the contained filters shall be run or not
};
}}
