/*************************************************************************\
   Copyright 2014 Institute of Industrial and Control Engineering (IOC)
                 Universitat Politecnica de Catalunya
                 BarcelonaTech
    All Rights Reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the
    Free Software Foundation, Inc.,
    59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 \*************************************************************************/

/* Author: Nestor Garcia Hidalgo */


#include <kautham/planner/plannerDataIO.h>

#include <pugixml.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <vector>
#include <utility>


std::string toString(const bool value) {
    return value? "true" : "false";
}


template<typename T>
std::string toString(const T &value) {
    return static_cast<std::ostringstream&>((std::ostringstream() << std::dec << value)).str();
}


template<typename T>
std::string toString(const T &begin, const T &end, const std::string &sep = " ") {
    std::string str;
    bool first = true;
    for (T it = begin; it != end; ++it) {
        if (!first) str.append(sep);
        str.append(toString(*it));
        first = false;
    }

    return str;
}


template<typename T>
bool fromString(T &value, const std::string &str) {
    try {
        value = boost::lexical_cast<T>(str);
        return true;
    } catch(...) {
        return false;
    }
}


template<typename T>
bool fromString(std::vector<T> &values, const std::string &str, const std::string &sep = " ") {
    values.clear();

    //Split sentence
    std::vector<std::string> tokens;
    boost::split(tokens,str,boost::is_any_of(sep),boost::token_compress_on);

    //Fill vector
    for (std::vector<std::string>::const_iterator token(tokens.begin());
         token != tokens.end(); ++token) {
        T value;
        if (!fromString(value,*token)) {
            values.clear();

            return false;
        }

        values.push_back(value);
    }

    return true;
}


void copyToReals(const ompl::control::ControlSpacePtr &cs,
                 std::vector<double> &reals, const ompl::control::Control *source) {
    ompl::control::Control *tmp = cs->allocControl();
    cs->copyControl(tmp,source);
    for (std::size_t i = 0; cs->getValueAddressAtIndex(tmp,i) != nullptr; ++i) {
        reals.push_back(*cs->getValueAddressAtIndex(tmp,i));
    }
    cs->freeControl(tmp);
}

void copyFromReals(const ompl::control::ControlSpacePtr &cs,
                   ompl::control::Control *destination, const std::vector<double> &reals) {
    assert(cs->getValueAddressAtIndex(destination,reals.size()) == nullptr);
    for (std::size_t i = 0; i < reals.size(); ++i) {
        assert(cs->getValueAddressAtIndex(destination,i) != nullptr);
        *cs->getValueAddressAtIndex(destination,i) = reals[i];
    }
}


bool savePlannerData(ompl::base::PlannerData *pdata, pugi::xml_document &doc,
                     const ompl::base::OptimizationObjective *opt) {
    //Compute edge weights
    if (opt == nullptr) {
        pdata->computeEdgeWeights();
    } else {
        pdata->computeEdgeWeights(*opt);
    }

    //Get ControlSpaceInformation, if any
    const ompl::control::SpaceInformation *siC = dynamic_cast
            <ompl::control::SpaceInformation*>(pdata->getSpaceInformation().get());

    //Create document
    pugi::xml_node node = doc.append_child("PlannerData");
    node.append_attribute("hasControls").set_value(pdata->hasControls());
    pugi::xml_node vertices = (pdata->numVertices() == 0)? pugi::xml_node() : node.append_child("Vertices");
    pugi::xml_node edges = (pdata->numEdges() == 0)? pugi::xml_node() : node.append_child("Edges");
    pugi::xml_node properties = pdata->properties.empty()? pugi::xml_node() : node.append_child("Properties");

    //Add vertices and edges
    for (unsigned int i = 0; i < pdata->numVertices(); ++i) {
        //Add i-th vertex
        const ompl::base::PlannerDataVertex vertex = pdata->getVertex(i);
        if (vertex == ompl::base::PlannerData::NO_VERTEX) {
            std::cout << "Error when obtaining " << i << "-th vertex." << std::endl;

            return false;
        }
        std::vector<double> coords;
        pdata->getSpaceInformation()->getStateSpace()->copyToReals(coords,vertex.getState());
        node = vertices.append_child("Vertex");
        node.append_attribute("state").set_value(toString(coords.begin(),coords.end()).c_str());
        node.append_attribute("tag").set_value(vertex.getTag());
        if (pdata->isStartVertex(i)) {
            node.append_attribute("type").set_value("start");
        } else if (pdata->isGoalVertex(i)) {
            node.append_attribute("type").set_value("goal");
        }

        //Add edges coming out from i-th vertex
        std::vector<unsigned int> edgeList;
        pdata->getEdges(i,edgeList);
        for (std::vector<unsigned int>::const_iterator j = edgeList.begin();
             j != edgeList.end(); ++j) {
            ompl::base::Cost weight;
            pdata->getEdgeWeight(i,*j,&weight);
            node = edges.append_child("Edge");
            node.append_attribute("v1").set_value(i);
            node.append_attribute("v2").set_value(*j);
            node.append_attribute("weight").set_value(weight.value());
            if (pdata->hasControls() && pdata->getEdge(i,*j) != ompl::base::PlannerData::NO_EDGE) {
                const ompl::control::PlannerDataEdgeControl *edge = dynamic_cast
                        <ompl::control::PlannerDataEdgeControl*>(&pdata->getEdge(i,*j));
                if (edge != nullptr && siC != nullptr) {
                    coords.clear();
                    copyToReals(siC->getControlSpace(),coords,edge->getControl());
                    node.append_attribute("controls").set_value(toString(coords.begin(),coords.end()).c_str());
                    node.append_attribute("duration").set_value(edge->getDuration());
                } else {
                    std::cout << "Error when obtaining edge from " << i << "-th vertex to "
                              << ((unsigned int)*j) << "-th vertex." << std::endl;

                    return false;
                }
            }
        }
    }

    //Add properties
    for (std::map<std::string,std::string>::const_iterator property = pdata->properties.begin();
         property != pdata->properties.end(); ++property) {
        node = properties.append_child("Property");
        node.append_attribute("key").set_value(property->first.c_str());
        node.append_attribute("value").set_value(property->second.c_str());
    }

    return true;
}


bool savePlannerData(ompl::base::PlannerData *pdata, const std::string &path,
                     const ompl::base::OptimizationObjective *opt) {
    pugi::xml_document doc;
    if (!savePlannerData(pdata,doc,opt)) return false;

    return doc.save_file(path.c_str());
}


bool savePlannerData(ompl::base::PlannerData *pdata, std::ostream &stream,
                     const ompl::base::OptimizationObjective *opt) {
    pugi::xml_document doc;
    if (!savePlannerData(pdata,doc,opt)) return false;

    doc.save(stream);

    return true;
}


ompl::base::PlannerData *loadPlannerData(const ompl::base::SpaceInformationPtr &si,
                                         const pugi::xml_document &doc) {
    //Check PlannerData node
    bool hasControls;
    if (!doc.child("PlannerData").attribute("hasControls") ||
            !fromString(hasControls,doc.child("PlannerData").attribute("hasControls").as_string())) {
        std::cout << "Error when creating PlannerData: invalid PlannerData node." << std::endl;

        return nullptr;
    }

    //Get ControlSpaceInformation, if any
    ompl::control::SpaceInformation *siC = dynamic_cast<ompl::control::SpaceInformation*>(si.get());

    //Create PlannerData
    ompl::base::PlannerDataPtr pdata;
    if (hasControls) {
        if (siC == nullptr) {
            std::cout << "Error when creating PlannerData: expected ompl::control::SpaceInformation." << std::endl;

            return nullptr;
        }

        pdata = ompl::base::PlannerDataPtr(new ompl::control::PlannerData
                                           (ompl::control::SpaceInformationPtr(siC)));
    } else {
        pdata = ompl::base::PlannerDataPtr(new ompl::base::PlannerData
                                           (ompl::base::SpaceInformationPtr(si)));
    }

    //Get vertices
    std::vector<ompl::base::State*> states;
    const unsigned int dim = si->getStateSpace()->getValueLocations().size();
    for (pugi::xml_node node = doc.child("PlannerData").child("Vertices").child("Vertex");
         node; node.next_sibling("Vertex")) {
        //Check state
        if (!node.attribute("state")) {
            std::cout << "Error when creating PlannerData: " << states.size()+1
                      << "-th vertex node has no state." << std::endl;

            si->freeStates(states);
            return nullptr;
        }

        //Get state
        ompl::base::State *state = si->allocState();
        states.push_back(state);
        std::vector<double> coords;
        if (!fromString(coords,node.attribute("state").as_string()) || coords.size() != dim) {
            std::cout << "Error when creating PlannerData: State of " << states.size()+1
                      << "-th vertex node should have dimension " << dim
                      << " but has dimension " << coords.size() << "." << std::endl;

            si->freeStates(states);
            return nullptr;
        }
        si->getStateSpace()->copyFromReals(state,coords);

        //Check tag
        if (!node.attribute("tag")) {
            std::cout << "Error when creating PlannerData: " << states.size()+1
                      << "-th vertex node has no tag." << std::endl;

            si->freeStates(states);
            return nullptr;
        }

        //Get tag
        int tag;
        if (!fromString(tag,node.attribute("tag").as_string())) {
            std::cout << "Error when creating PlannerData: " << states.size()+1
                      << "-th vertex node has an invalid tag." << std::endl;

            si->freeStates(states);
            return nullptr;
        }

        //Get vertex
        const ompl::base::PlannerDataVertex vertex(state,tag);
        unsigned int index;
        if (!node.attribute("type")) {
            index = pdata->addVertex(vertex);
        } else {
            if (strcmp(node.attribute("type").as_string(),"start") == 0) {
                index = pdata->addStartVertex(vertex);
            } else if (strcmp(node.attribute("type").as_string(),"goal") == 0) {
                index = pdata->addGoalVertex(vertex);
            } else {
                std::cout << "Error when creating PlannerData: " << states.size()+1
                          << "-th vertex node has an invalid type." << std::endl;

                si->freeStates(states);
                return nullptr;
            }
        }
        if (index == ompl::base::PlannerData::INVALID_INDEX) {
            std::cout << "Error when adding " << states.size()+1 << "-th vertex." << std::endl;

            si->freeStates(states);
            return nullptr;
        }
    }

    //Get edges
    std::vector<ompl::control::Control*> controls;
    unsigned int dimC = 0;
    if (hasControls) {
        ompl::control::Control *control = siC->allocControl();
        for ( ; siC->getControlSpace()->getValueAddressAtIndex(control,dimC) != nullptr; ++dimC);
        siC->freeControl(control);
    }
    for (pugi::xml_node node = doc.child("PlannerData").child("Edges").child("Edge");
         node; node.next_sibling("Edge")) {
        //Check vertex 1
        if (!node.attribute("v1")) {
            std::cout << "Error when creating PlannerData: " << pdata->numEdges()+1
                      << "-th edge node has no vertex 1." << std::endl;

            si->freeStates(states);
            for (std::vector<ompl::control::Control*>::const_iterator control = controls.begin();
                 control != controls.end(); ++control) siC->freeControl(*control);
            return nullptr;
        }

        //Get vertex 1
        unsigned int v1;
        if (!fromString(v1,node.attribute("v1").as_string())) {
            std::cout << "Error when creating PlannerData: " << pdata->numEdges()+1
                      << "-th edge node has an invalid vertex 1." << std::endl;

            si->freeStates(states);
            for (std::vector<ompl::control::Control*>::const_iterator control = controls.begin();
                 control != controls.end(); ++control) siC->freeControl(*control);
            return nullptr;
        }

        //Check vertex 2
        if (!node.attribute("v2")) {
            std::cout << "Error when creating PlannerData: " << pdata->numEdges()+1
                      << "-th edge node has no vertex 2." << std::endl;

            si->freeStates(states);
            for (std::vector<ompl::control::Control*>::const_iterator control = controls.begin();
                 control != controls.end(); ++control) siC->freeControl(*control);
            return nullptr;
        }

        //Get vertex 2
        unsigned int v2;
        if (!fromString(v2,node.attribute("v2").as_string())) {
            std::cout << "Error when creating PlannerData: " << pdata->numEdges()+1
                      << "-th edge node has an invalid vertex 2." << std::endl;

            si->freeStates(states);
            for (std::vector<ompl::control::Control*>::const_iterator control = controls.begin();
                 control != controls.end(); ++control) siC->freeControl(*control);
            return nullptr;
        }

        //Check weight
        if (!node.attribute("weight")) {
            std::cout << "Error when creating PlannerData: " << pdata->numEdges()+1
                      << "-th edge node has no weight." << std::endl;

            si->freeStates(states);
            for (std::vector<ompl::control::Control*>::const_iterator control = controls.begin();
                 control != controls.end(); ++control) siC->freeControl(*control);
            return nullptr;
        }

        //Get weight
        double weight;
        if (!fromString(weight,node.attribute("weight").as_string())) {
            std::cout << "Error when creating PlannerData: " << pdata->numEdges()+1
                      << "-th edge node has an invalid weight." << std::endl;

            si->freeStates(states);
            for (std::vector<ompl::control::Control*>::const_iterator control = controls.begin();
                 control != controls.end(); ++control) siC->freeControl(*control);
            return nullptr;
        }

        //Get edge
        ompl::base::PlannerDataEdge *edge;
        if (hasControls) {
            //Check control
            if (!node.attribute("control")) {
                std::cout << "Error when creating PlannerData: " << states.size()+1
                          << "-th vertex node has no control." << std::endl;

                si->freeStates(states);
                for (std::vector<ompl::control::Control*>::const_iterator control = controls.begin();
                     control != controls.end(); ++control) siC->freeControl(*control);
                return nullptr;
            }

            //Get control
            ompl::control::Control *control = siC->allocControl();
            controls.push_back(control);
            std::vector<double> coords;
            if (!fromString(coords,node.attribute("control").as_string()) || coords.size() != dimC) {
                std::cout << "Error when creating PlannerData: Control of " << controls.size()+1
                          << "-th edge node should have dimension " << dimC
                          << " but has dimension " << coords.size() << "." << std::endl;

                si->freeStates(states);
                for (std::vector<ompl::control::Control*>::const_iterator control = controls.begin();
                     control != controls.end(); ++control) siC->freeControl(*control);
                return nullptr;
            }
            copyFromReals(siC->getControlSpace(),control,coords);

            //Check duration
            if (!node.attribute("duration")) {
                std::cout << "Error when creating PlannerData: " << pdata->numEdges()+1
                          << "-th edge node has no duration." << std::endl;

                si->freeStates(states);
                for (std::vector<ompl::control::Control*>::const_iterator control = controls.begin();
                     control != controls.end(); ++control) siC->freeControl(*control);
                return nullptr;
            }

            //Get duration
            double duration;
            if (!fromString(duration,node.attribute("duration").as_string())) {
                std::cout << "Error when creating PlannerData: " << pdata->numEdges()+1
                          << "-th edge node has an invalid duration." << std::endl;

                si->freeStates(states);
                for (std::vector<ompl::control::Control*>::const_iterator control = controls.begin();
                     control != controls.end(); ++control) siC->freeControl(*control);
                return nullptr;
            }

            edge = new ompl::control::PlannerDataEdgeControl(control,duration);
        } else {
            edge = new ompl::base::PlannerDataEdge();
        }

        pdata->addEdge(pdata->vertexIndex(states.at(v1)),
                       pdata->vertexIndex(states.at(v2)),*edge,ompl::base::Cost(weight));
        delete edge;
    }

    //Get properties
    for (pugi::xml_node node = doc.child("PlannerData").child("Properties").child("Property");
         node; node.next_sibling("Property")) {
        //Check key
        if (!node.attribute("key")) {
            std::cout << pdata->properties.size()+1 << "-th property node has no key." << std::endl;

            si->freeStates(states);
            for (std::vector<ompl::control::Control*>::const_iterator control = controls.begin();
                 control != controls.end(); ++control) siC->freeControl(*control);
            return nullptr;
        }

        //Check value
        if (!node.attribute("value")) {
            std::cout << pdata->properties.size()+1 << "-th property node has no value." << std::endl;

            si->freeStates(states);
            for (std::vector<ompl::control::Control*>::const_iterator control = controls.begin();
                 control != controls.end(); ++control) siC->freeControl(*control);
            return nullptr;
        }

        //Set property
        if (!pdata->properties.insert(std::pair<std::string,std::string>
                                      (node.attribute("key").as_string(),
                                       node.attribute("value").as_string())).second) {
            std::cout << "Error when creating PlannerData: Two properties with key ["
                      << node.attribute("key").as_string() << "]." << std::endl;

            si->freeStates(states);
            for (std::vector<ompl::control::Control*>::const_iterator control = controls.begin();
                 control != controls.end(); ++control) siC->freeControl(*control);
            return nullptr;
        }
    }
    pdata->decoupleFromPlanner();//So that plannerData is the owner of the states

    si->freeStates(states);
    for (std::vector<ompl::control::Control*>::const_iterator control = controls.begin();
         control != controls.end(); ++control) siC->freeControl(*control);
    return pdata.get();
}


ompl::base::PlannerData *loadPlannerData(const ompl::base::SpaceInformationPtr &si,
                                         const std::string &path) {
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(path.c_str());
    if (!result) {
        std::cout << "PlannerData file [" << path << "] couldn't be parsed." << std::endl
                  << "Error: " << result.description() << std::endl
                  << "Last successfully parsed character: " << result.offset << std::endl;

        return nullptr;
    }

    return loadPlannerData(si,doc);
}


ompl::base::PlannerData *loadPlannerData(const ompl::base::SpaceInformationPtr &si,
                                         std::istream &stream) {
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load(stream);
    if (!result) {
        std::cout << "PlannerData stream couldn't be parsed." << std::endl
                  << "Error: " << result.description() << std::endl
                  << "Last successfully parsed character: " << result.offset << std::endl;

        return nullptr;
    }

    return loadPlannerData(si,doc);
}
