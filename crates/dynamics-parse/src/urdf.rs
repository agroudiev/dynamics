//! Parser for the URDF (Unified Robot Description Format) file format.
//!
//! ## Overview
//! This module provides functionality to parse URDF files and build corresponding [`Model`] and [`GeometryModel`] objects.
//!
//! ## Joints
//! The following joint types are supported:
//! - Fixed
//! - Revolute
//! - Continuous
//! - Prismatic
//!
//! ## Meshes
//! The parser supports mesh geometries in any format. The visualizers used might have limitations on the supported formats.
//! Mesh paths can be resolved in three ways:
//! - using absolute paths;
//! - using ROS's `package://` (or `package:///`) syntax, as long as the package name is present in the environment variable `ROS_PACKAGE_PATH`;
//! - using a user-provided package directory, passed as an argument to the parsing function.
//!
//! ## Tested models
//! The parser has been tested at least on all models from [example-robot-data](https://github.com/Gepetto/example-robot-data/).
//! However, some URDF features are still missing, so some models might not be parsed correctly yet.

#![allow(clippy::too_many_arguments)] // TODO: refactor functions

use crate::errors::ParseError;
use collider_rs::mesh::Mesh;
use collider_rs::shape::Cuboid;
use collider_rs::shape::{Cylinder, ShapeWrapper, Sphere};
use dynamics_inertia::inertia::Inertia;
use dynamics_joint::continuous::JointModelContinuous;
use dynamics_joint::joint::JointWrapper;
use dynamics_joint::prismatic::JointModelPrismatic;
use dynamics_joint::revolute::JointModelRevolute;
use dynamics_model::frame::{Frame, FrameType};
use dynamics_model::model::Model;
use dynamics_model::{
    geometry_model::GeometryModel, geometry_object::GeometryObject, model::WORLD_ID,
};
use dynamics_spatial::color::Color;
use dynamics_spatial::motion::SpatialRotation;
use dynamics_spatial::se3::{ActSE3, SE3};
use dynamics_spatial::symmetric3::Symmetric3;
use dynamics_spatial::vector3d::Vector3D;
use nalgebra::Vector3;
use roxmltree::{Document, Node};
use std::collections::HashSet;
use std::path::Path;
use std::{collections::HashMap, fs, str::FromStr};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Type of geometry being parsed.
enum GeometryType {
    Collision,
    Visual,
}

/// Parses a URDF file and builds the corresponding [`Model`] and [`GeometryModel`].
///
/// # Arguments
///
/// * `filepath` - The path to the URDF file.
/// * `package_dir` - The path to the folder containing the mesh files referenced in the URDF.
///
/// # Returns
///
/// A 3-tuple containing the [`Model`], a collision [`GeometryModel`], and a visualization [`GeometryModel`] objects if successful.
///
/// Returns a [`ParseError`] if there is an error during parsing.
pub fn build_models_from_urdf(
    filepath: &str,
    package_dir: Option<&str>,
) -> Result<(Model, GeometryModel, GeometryModel), ParseError> {
    let contents =
        fs::read_to_string(filepath).map_err(|e| ParseError::IoError(e, filepath.to_string()))?;
    let doc = Document::parse(&contents).map_err(ParseError::XmlError)?;

    // identify the robot node
    let robot_node = doc
        .descendants()
        .find(|n| n.tag_name().name() == "robot")
        .ok_or(ParseError::NoRobotTag)?;
    let robot_name = robot_node.attribute("name").unwrap_or("").to_string();

    // elements will sequentially be added to the models
    let mut model = Model::new(robot_name);
    let mut coll_model = GeometryModel::new();
    let mut viz_model = GeometryModel::new();

    // start by parsing materials at the root
    let mut materials = parse_root_materials(&robot_node)?;

    // find root nodes (links without parents)
    let root_nodes = find_root_nodes(&robot_node)?;

    // sort root nodes in alphabetical order
    let mut root_nodes: Vec<Node> = root_nodes.into_iter().collect();
    root_nodes.sort_by_key(|n| n.attribute("name").unwrap_or(""));

    // recursively parse from the root nodes
    for node in root_nodes {
        parse_node(
            &robot_node,
            node,
            &robot_node,
            WORLD_ID,
            true,
            &mut model,
            &mut coll_model,
            &mut viz_model,
            &mut materials,
            package_dir,
        )?;
    }

    Ok((model, coll_model, viz_model))
}

/// Parse the given node and call itself recursively on its children.
///
/// The steps are:
/// - Depending on the type of node (link or joint), call the corresponding parse function.
/// - Find all children nodes (joints for links, child links for joints).
/// - Sort the children nodes in alphabetical order.
/// - Recursively call `parse_node` on each child node.
///
/// # Arguments:
/// - `robot_node`: The root robot node containing all links and joints.
/// - `node`: The current node to parse.
/// - `parent_node`: The parent node of the current node.
/// - `parent_joint_id`: The ID of the parent joint in the model.
/// - `is_root`: Whether the current node is a root node.
/// - `model`: The model being built.
/// - `coll_model`: The collision geometry model being built.
/// - `viz_model`: The visualization geometry model being built.
/// - `materials`: The map of materials defined in the robot.
/// - `package_dir`: The path to the folder containing the mesh files referenced in the URDF.
///
/// # Returns:
/// - `Result<(), ParseError>`: `Ok` if successful, or a [`ParseError`] if an error occurs.
fn parse_node(
    robot_node: &Node,
    node: Node,
    parent_node: &Node,
    parent_joint_id: usize,
    is_root: bool,
    model: &mut Model,
    coll_model: &mut GeometryModel,
    viz_model: &mut GeometryModel,
    materials: &mut HashMap<String, Color>,
    package_dir: Option<&str>,
) -> Result<(), ParseError> {
    let node_name = node.attribute("name").unwrap_or("");
    let mut new_parent_joint_id = parent_joint_id;

    // parse the current node and extract its children
    let children = match node.tag_name().name() {
        "link" => {
            parse_link(
                robot_node,
                node,
                parent_node,
                parent_joint_id,
                is_root,
                model,
                coll_model,
                viz_model,
                materials,
                package_dir,
            )?;

            // find all joints that have this link as parent
            let mut children = Vec::new();

            for joint_node in robot_node.children().filter(|n| n.has_tag_name("joint")) {
                // if this joint:
                // - has a parent tag
                // - which is a link
                // - and this link matches the current node name
                if let Some(parent_node) = joint_node.children().find(|n| n.has_tag_name("parent"))
                    && let Ok(parent_link_name) = extract_parameter::<String>("link", &parent_node)
                    && parent_link_name == node_name
                {
                    // then add this joint node to the children to be parsed next
                    children.push(joint_node);
                }
            }
            children
        }

        // parse joints
        "joint" => {
            new_parent_joint_id =
                parse_joint(robot_node, node, parent_node, parent_joint_id, model)?;

            // add the eventual child link to the children to be parsed next
            if let Some(child_node) = node.children().find(|n| n.has_tag_name("child")) {
                // find the name of the child link
                let child_link_name = extract_parameter::<String>("link", &child_node)?;

                // find the link node in the robot node
                if let Some(link_node) = robot_node.children().find(|n| {
                    n.has_tag_name("link") && n.attribute("name").unwrap_or("") == child_link_name
                }) {
                    vec![link_node]
                } else {
                    // child link not found in the robot description
                    return Err(ParseError::UnknownLinkName(child_link_name));
                }
            } else {
                vec![]
            }
        }

        // we ignore empty lines and tags not used in simulation
        "" | "gazebo" | "transmission" => {
            vec![]
        }

        // unknown tag
        _ => {
            return Err(ParseError::UnknownTag(node.tag_name().name().to_string()));
        }
    };

    // sort the children in alphabetical order
    let mut children: Vec<Node> = children.into_iter().collect();
    children.sort_by_key(|n| n.attribute("name").unwrap_or(""));

    // recursively parse children
    for child_node in children {
        parse_node(
            robot_node,
            child_node,
            &node,
            new_parent_joint_id,
            false,
            model,
            coll_model,
            viz_model,
            materials,
            package_dir,
        )?;
    }
    Ok(())
}

/// Finds the root nodes (links without parents) in the robot node.
///
/// It starts by collecting all link nodes, then removes those that are children of joints.
///
/// # Arguments:
/// - `robot_node`: The root robot node containing all links and joints.
///
/// # Returns:
/// - `Result<HashSet<Node>, ParseError>`: A set of root link nodes if successful, or a [`ParseError`] if an error occurs.
fn find_root_nodes<'a>(robot_node: &'a Node) -> Result<HashSet<Node<'a, 'a>>, ParseError> {
    // collect all link nodes
    let mut parent_links: HashSet<Node> = robot_node
        .children()
        .filter(|n| n.has_tag_name("link"))
        .collect();

    // remove all links that are children of joints
    for joint_node in robot_node.children().filter(|n| n.has_tag_name("joint")) {
        // find the parent link
        if let Some(child_node) = joint_node.children().find(|n| n.has_tag_name("child")) {
            let child_link_name = extract_parameter::<String>("link", &child_node)?;
            // remove the parent link from the set
            parent_links.retain(|link_node| {
                let link_name = link_node.attribute("name").unwrap_or("");
                link_name != child_link_name
            });
        }
    }

    Ok(parent_links)
}

/// Parses a joint from the URDF file and adds it to the model.
///
/// It extracts the joint name, type, origin, axis, and limits, and creates the corresponding joint model.
/// If the joint is fixed, no joint model is created, only a fixed frame is added.
/// In any case, a frame (of type either fixed or joint) is added for the joint.
///
/// # Arguments:
/// - `robot_node`: The root robot node containing all links and joints.
/// - `joint_node`: The joint node to parse.
/// - `parent_node`: The parent node of the joint.
/// - `parent_joint_id`: The ID of the parent joint in the model.
/// - `model`: The model being built.
///
/// # Returns:
/// - `Result<usize, ParseError>`: The ID of the newly added joint if successful, or a [`ParseError`] if an error occurs.
fn parse_joint(
    robot_node: &Node,
    joint_node: Node,
    parent_node: &Node,
    parent_joint_id: usize,
    model: &mut Model,
) -> Result<usize, ParseError> {
    // extract the name and type of joint
    let joint_name = joint_node
        .attribute("name")
        .ok_or(ParseError::NameMissing(format!("{joint_node:#?}")))?
        .to_string();
    let joint_type = joint_node
        .attribute("type")
        .ok_or(ParseError::MissingParameter("type".to_string()))?;

    // get the parent frame
    let parent_frame_id = get_parent_frame(parent_node, robot_node, model)?;
    let parent_frame = &model.frames[parent_frame_id];

    // compute the placement of the joint
    let link_origin = parse_origin(&joint_node)?;
    let placement = parent_frame.placement * link_origin;

    let new_joint_id = match joint_type {
        // if the joint is fixed, we create a fixed frame
        "fixed" => {
            let frame = Frame::new(
                joint_name.clone(),
                parent_joint_id,
                parent_frame_id,
                placement,
                FrameType::Fixed,
                Inertia::zeros(),
            );
            let _ = model.add_frame(frame, false);

            // we return the parent joint id as there is no new joint
            Ok(parent_joint_id)
        }

        // if the joint is continuous, we create a revolute joint without limits
        "continuous" => {
            let axis = parse_axis(joint_node)?;
            let mut joint_model = JointModelContinuous::new(axis);

            // we extract the limit of the joint
            if let Some(limit_node) = joint_node.children().find(|n| n.has_tag_name("limit")) {
                let effort = extract_parameter::<f64>("effort", &limit_node)?;
                joint_model.limits.effort[0] = effort;

                let velocity = extract_parameter::<f64>("velocity", &limit_node)?;
                joint_model.limits.velocity[0] = velocity;
            }

            model.add_joint(
                parent_joint_id,
                JointWrapper::continuous(joint_model),
                placement,
                joint_name.clone(),
            )
        }

        // if the joint is revolute, we create a revolute joint
        "revolute" => {
            let axis = parse_axis(joint_node)?;

            // we extract the limit of the joint
            let limit_node = joint_node
                .children()
                .find(|n| n.has_tag_name("limit"))
                .ok_or(ParseError::MissingParameter("limit".to_string()))?;

            // TODO: extract dynamics (damping, ...)

            let mut joint_model = JointModelRevolute::new(axis);

            // optional parameters
            if let Ok(lower) = extract_parameter::<f64>("lower", &limit_node) {
                joint_model.limits.min_configuration[0] = lower;
            }
            if let Ok(upper) = extract_parameter::<f64>("upper", &limit_node) {
                joint_model.limits.max_configuration[0] = upper;
            }

            // required parameters
            let effort = extract_parameter::<f64>("effort", &limit_node)?;
            joint_model.limits.effort[0] = effort;

            let velocity = extract_parameter::<f64>("velocity", &limit_node)?;
            joint_model.limits.velocity[0] = velocity;

            model.add_joint(
                parent_joint_id,
                JointWrapper::revolute(joint_model),
                placement,
                joint_name.clone(),
            )
        }

        "prismatic" => {
            let axis = parse_axis(joint_node)?;

            // we extract the limit of the joint
            let limit_node = joint_node
                .children()
                .find(|n| n.has_tag_name("limit"))
                .ok_or(ParseError::MissingParameter("limit".to_string()))?;

            // TODO: extract dynamics (damping, ...)

            let mut joint_model = JointModelPrismatic::new(axis);

            // optional parameters
            if let Ok(lower) = extract_parameter::<f64>("lower", &limit_node) {
                joint_model.limits.min_configuration[0] = lower;
            }
            if let Ok(upper) = extract_parameter::<f64>("upper", &limit_node) {
                joint_model.limits.max_configuration[0] = upper;
            }

            // required parameters
            let effort = extract_parameter::<f64>("effort", &limit_node)?;
            joint_model.limits.effort[0] = effort;

            let velocity = extract_parameter::<f64>("velocity", &limit_node)?;
            joint_model.limits.velocity[0] = velocity;

            model.add_joint(
                parent_joint_id,
                JointWrapper::prismatic(joint_model),
                placement,
                joint_name.clone(),
            )
        }

        _ => return Err(ParseError::UnknownJointType(joint_type.to_string())),
    }
    .map_err(ParseError::ModelError)?;

    // if the joint is not fixed, we also add a frame
    if joint_type != "fixed" {
        let frame = Frame::new(
            joint_name,
            new_joint_id,
            parent_frame_id,
            SE3::identity(),
            FrameType::Joint,
            Inertia::zeros(),
        );
        model
            .add_frame(frame, true)
            .map_err(ParseError::ModelError)?;
    }

    Ok(new_joint_id)
}

/// Parses an axis defined in the URDF file, given a joint node.
///
/// # Arguments:
/// - `joint_node`: The joint node containing the axis definition.
///
/// # Returns:
/// - `Result<Vector3D, ParseError>`: The parsed axis as a `Vector3D` if successful, or a [`ParseError`] if an error occurs.
fn parse_axis(joint_node: Node) -> Result<Vector3D, ParseError> {
    if let Some(axis_node) = joint_node.children().find(|n| n.has_tag_name("axis")) {
        let axis_values = extract_parameter_list::<f64>("xyz", &axis_node, Some(3))?;
        Ok(Vector3D::new(
            axis_values[0],
            axis_values[1],
            axis_values[2],
        ))
    } else {
        // default axis if not specified
        Ok(Vector3D::new(1.0, 0.0, 0.0))
    }
}

/// Parses all materials defined at the root of the robot node.
///
/// # Arguments:
/// - `robot_node`: The root robot node containing material definitions.
///
/// # Returns:
/// - A map from material names to their corresponding `Color` objects if successful, or a [`ParseError`] if an error occurs.
fn parse_root_materials(robot_node: &Node) -> Result<HashMap<String, Color>, ParseError> {
    let mut materials = HashMap::new();
    for material_node in robot_node.children().filter(|n| n.has_tag_name("material")) {
        parse_material(material_node, &mut materials)?;
    }
    Ok(materials)
}

/// Parses a link node.
///
/// This functions extracts the link's inertia, adds a frame for the link, and parses its collision and visual geometries.
/// The geometries are added to the provided collision and visualization geometry models.
/// To each link is associated exactly one frame in the model.
///
/// # Arguments:
/// - `robot_node`: The root robot node containing all links and joints.
/// - `node`: The link node to parse.
/// - `parent_node`: The XML parent node of the link.
/// - `parent_joint_id`: The ID of the parent joint in the model.
/// - `is_root`: Whether the link is a root link.
/// - `model`: The model being built.
/// - `coll_model`: The collision geometry model being built.
/// - `viz_model`: The visualization geometry model being built.
/// - `materials`: The map of materials defined in the robot.
/// - `package_dir`: The path to the folder containing the mesh files referenced in the URDF.
///
/// # Returns:
/// - `Result<(), ParseError>`: `Ok` if successful, or a [`ParseError`] if an error occurs.
fn parse_link(
    robot_node: &Node,
    node: Node,
    parent_node: &Node,
    parent_joint_id: usize,
    is_root: bool,
    model: &mut Model,
    coll_model: &mut GeometryModel,
    viz_model: &mut GeometryModel,
    materials: &mut HashMap<String, Color>,
    package_dir: Option<&str>,
) -> Result<(), ParseError> {
    let link_name = node.attribute("name").unwrap_or("").to_string();
    let parent_frame_id = get_parent_frame(parent_node, robot_node, model)?;

    // parse the inertial node
    let link_inertia = parse_inertia(node, &link_name)?;

    // inertia associated with the new frame
    // if this is the root link, we put the inertia in the frame
    // otherwise, we associate the inertia to the joint (in model.inertias)
    let frame_inertia = if is_root {
        link_inertia
    }
    // if the parent joint type is fixed, we put the inertia in the parent's frame
    else if let Some(parent_joint_type) = parent_node.attribute("type")
        && parent_joint_type == "fixed"
    {
        model.frames[parent_frame_id].inertia += link_inertia.clone(); // TODO: check if this clone can be avoided
        model.inertias[parent_joint_id] +=
            link_inertia.act(&model.frames[parent_frame_id].placement);
        Inertia::zeros()
    } else {
        model.inertias[parent_joint_id] +=
            link_inertia.act(&model.frames[parent_frame_id].placement);
        Inertia::zeros()
    };

    // compute the placement of the link frame
    let parent_frame = &model.frames[parent_frame_id];
    let link_placement = parent_frame.placement * parse_origin(&node)?;

    // add a frame for the link
    let frame = Frame::new(
        link_name.to_string(),
        parent_joint_id,
        parent_frame_id,
        link_placement,
        FrameType::Body,
        frame_inertia,
    );
    let new_frame_id = model
        .add_frame(frame, true)
        .map_err(ParseError::ModelError)?;

    // parse the collision node
    for (i, collision_node) in node
        .children()
        .filter(|n| n.has_tag_name("collision"))
        .enumerate()
    {
        let geom_obj = parse_geometry(
            format!("{link_name}_{i}"),
            &collision_node,
            parent_joint_id,
            new_frame_id,
            model,
            materials,
            package_dir,
            GeometryType::Collision,
        )?;
        coll_model.add_geometry_object(geom_obj);
    }

    // parse the visual node
    for (i, visual_node) in node
        .children()
        .filter(|n| n.has_tag_name("visual"))
        .enumerate()
    {
        let geom_obj = parse_geometry(
            format!("{link_name}_{i}"),
            &visual_node,
            parent_joint_id,
            new_frame_id,
            model,
            materials,
            package_dir,
            GeometryType::Visual,
        )?;
        viz_model.add_geometry_object(geom_obj);
    }

    Ok(())
}

/// Retrieves the parent frame index from the parent node.
fn get_parent_frame(
    parent_node: &Node,
    robot_node: &Node,
    model: &Model,
) -> Result<usize, ParseError> {
    // extract the name of the parent link
    let parent_name = parent_node
        .attribute("name")
        .ok_or(ParseError::NameMissing(format!("{parent_node:#?}")))?;
    let robot_name = robot_node
        .attribute("name")
        .ok_or(ParseError::NameMissing(format!("{robot_node:#?}")))?;

    // check if the parent is the world frame
    if parent_name == robot_name {
        Ok(WORLD_ID)
    }
    // else, retrieve the frame id from the model
    else {
        // compute the type of the parent node
        let parent_type = parent_node.attribute("type").unwrap_or("");
        let frame_type = match parent_type {
            "fixed" => FrameType::Fixed,
            "revolute" | "continuous" | "prismatic" => FrameType::Joint,
            _ => FrameType::Body,
        };
        model
            .get_frame_id(parent_name, Some(frame_type))
            .ok_or(ParseError::UnknownParent(parent_name.to_string()))
    }
}

/// Parses a material from the URDF file.
///
/// # Arguments:
/// - `node`: The material node to parse.
/// - `materials`: The map of materials to update.
///
/// # Returns:
/// - If successful, updates the `materials` map with the parsed material.
/// - Otherwise, returns a [`ParseError`].
fn parse_material(node: Node, materials: &mut HashMap<String, Color>) -> Result<(), ParseError> {
    // extract name
    let material_name = node
        .attribute("name")
        .ok_or(ParseError::NameMissing(format!("{node:#?}")))?
        .to_string();

    // extract and convert color
    let color_node = node
        .children()
        .find(|n| n.has_tag_name("color"))
        .ok_or(ParseError::MaterialWithoutColor(material_name.clone()))?;
    let rgba = extract_parameter_list::<f64>("rgba", &color_node, Some(4))?;
    let color = Color::new(rgba[0], rgba[1], rgba[2], rgba[3]);

    // TODO: handle texture

    materials.insert(material_name, color);
    Ok(())
}

/// Parses the inertia of a link from the URDF file.
///
/// # Arguments:
/// - `node`: The link node to parse.
/// - `link_name`: The name of the link (for error reporting).
///
/// # Returns:
/// - The parsed [`Inertia`] if successful.
/// - A [`ParseError`] if an error occurs.
fn parse_inertia(node: Node, link_name: &str) -> Result<Inertia, ParseError> {
    if let Some(inertial_node) = node.children().find(|n| n.has_tag_name("inertial")) {
        let mass_node = inertial_node
            .children()
            .find(|n| n.has_tag_name("mass"))
            .ok_or(ParseError::InertialWithoutMass(link_name.to_string()))?;
        let mass = extract_parameter::<f64>("value", &mass_node)?;

        let inertia_node = inertial_node
            .children()
            .find(|n| n.has_tag_name("inertia"))
            .ok_or(ParseError::InertialWithoutInertia(link_name.to_string()))?;

        let ixx = extract_parameter::<f64>("ixx", &inertia_node)?;
        let ixy = extract_parameter::<f64>("ixy", &inertia_node)?;
        let ixz = extract_parameter::<f64>("ixz", &inertia_node)?;
        let iyy = extract_parameter::<f64>("iyy", &inertia_node)?;
        let iyz = extract_parameter::<f64>("iyz", &inertia_node)?;
        let izz = extract_parameter::<f64>("izz", &inertia_node)?;
        let inertia_mat = Symmetric3::new(ixx, iyy, izz, ixy, ixz, iyz);

        let inertial_origin = if let Some(origin_node) =
            inertial_node.children().find(|n| n.has_tag_name("origin"))
        {
            // extract rpy and xyz (default to zeros if not specified)
            let rpy =
                extract_parameter_list::<f64>("rpy", &origin_node, Some(3)).unwrap_or(vec![0.0; 3]);
            let xyz =
                extract_parameter_list::<f64>("xyz", &origin_node, Some(3)).unwrap_or(vec![0.0; 3]);

            // construct the rotation and translation
            let rotation = SpatialRotation::from_euler_angles(rpy[0], rpy[1], rpy[2]);
            let translation = Vector3D::new(xyz[0], xyz[1], xyz[2]);

            SE3::from_parts(translation, rotation)
        } else {
            // defaults to identity if no origin is specified
            SE3::identity()
        };

        Ok(Inertia::new(
            mass,
            inertial_origin.translation(),
            inertia_mat.rotate(&inertial_origin.rotation()),
        ))
    } else {
        Ok(Inertia::zeros())
    }
}

/// Parses the geometry of a link from the URDF file.
/// Extracts the geometry shape and its parameters.
///
/// # Arguments:
/// - `link_name`: The name of the link (for error reporting).
/// - `node`: The geometry node to parse.
/// - `parent_joint_id`: The ID of the parent joint in the model.
/// - `parent_frame_id`: The ID of the parent frame in the model.
/// - `model`: The model being built.
/// - `materials`: The map of materials defined in the robot.
/// - `package_dir`: The path to the folder containing the mesh files referenced in the URDF.
/// - `geometry_type`: The type of geometry being parsed (collision or visual).
///
/// # Returns:
/// - The parsed [`GeometryObject`] if successful.
/// - A [`ParseError`] if an error occurs.
fn parse_geometry(
    link_name: String,
    node: &Node,
    parent_joint_id: usize,
    parent_frame_id: usize,
    model: &Model,
    materials: &mut HashMap<String, Color>,
    package_dir: Option<&str>,
    geometry_type: GeometryType,
) -> Result<GeometryObject, ParseError> {
    let geometry_node = node
        .children()
        .find(|n| n.has_tag_name("geometry"))
        .ok_or(ParseError::VisualWithoutGeometry(link_name.clone()))?;

    // extract the shape from the geometry node
    let shape: ShapeWrapper = if let Some(shape_node) =
        geometry_node.children().find(|n| n.has_tag_name("box"))
    {
        let size = extract_parameter_list::<f32>("size", &shape_node, Some(3))?;
        let half_extents = Vector3::new(size[0] / 2.0, size[1] / 2.0, size[2] / 2.0);
        Box::new(Cuboid::new(half_extents))
    } else if let Some(shape_node) = geometry_node
        .children()
        .find(|n| n.has_tag_name("cylinder"))
    {
        let radius = extract_parameter::<f32>("radius", &shape_node)?;
        let length = extract_parameter::<f32>("length", &shape_node)?;
        Box::new(Cylinder::new(radius, length / 2.0))
    } else if let Some(shape_node) = geometry_node.children().find(|n| n.has_tag_name("sphere")) {
        let radius = extract_parameter::<f32>("radius", &shape_node)?;
        Box::new(Sphere::new(radius))
    } else if let Some(mesh_node) = geometry_node.children().find(|n| n.has_tag_name("mesh")) {
        let filename = mesh_node
            .attribute("filename")
            .ok_or(ParseError::MissingParameter("filename".to_string()))?;

        let absolute_path = if let Some(filename) = filename.strip_prefix("package://") {
            // if the filename starts with "package:///", we remove the first slash
            let filename = filename.strip_prefix('/').unwrap_or(filename);

            // retrieve the package path in between "package://" and the first "/"
            let path_parts: Vec<&str> = filename.splitn(2, '/').collect();
            if path_parts.len() != 2 {
                return Err(ParseError::InvalidFilePath(format!(
                    "Invalid package path: {}",
                    filename
                )));
            }
            let package_name = path_parts[0];
            let relative_path = path_parts[1];

            // retrieve the package path from the 'ROS_PACKAGE_PATH' environment variable
            let ros_package_path = std::env::var("ROS_PACKAGE_PATH").map_err(|_| ParseError::InvalidFilePath(
                "'package://' was used but the ROS_PACKAGE_PATH environment variable is not set".to_string(),
            ))?;
            let package_paths: Vec<&str> = ros_package_path
                .split(':')
                .filter(|s| !s.is_empty())
                .collect();

            // search for the package in the package paths
            let mut package_path = None;
            for path in package_paths {
                // extract the last folder name in the path
                let path_obj = Path::new(path);

                // check if the folder name matches the package name
                if let Some(folder_name) = path_obj.file_name().and_then(|s| s.to_str())
                    && folder_name == package_name
                {
                    package_path = Some(path_obj.to_path_buf());
                    break;
                }
            }

            let package_path = package_path.ok_or(ParseError::InvalidFilePath(format!(
                "Package '{package_name}' not found in ROS_PACKAGE_PATH",
            )))?;

            // construct the absolute path with the mesh relative path
            package_path
                .join(relative_path)
                .to_str()
                .ok_or(ParseError::InvalidFilePath(format!(
                    "Invalide path: {}",
                    package_path.join(relative_path).display()
                )))?
                .to_string()
        } else if filename.starts_with('/') {
            filename.to_string()
        } else {
            // treat path as relative path from package_dir
            let package_dir = package_dir.ok_or(ParseError::InvalidFilePath(
                "'package_dir' must be provided to resolve mesh file paths".to_string(),
            ))?;
            let package_dir = Path::new(package_dir);
            package_dir
                .join(filename)
                .to_str()
                .ok_or(ParseError::InvalidFilePath(format!(
                    "Invalide path: {}",
                    package_dir.join(filename).display()
                )))?
                .to_string()
        };

        // check if the file exists
        if !std::path::Path::new(&absolute_path).exists() {
            return Err(ParseError::InvalidFilePath(format!(
                "Mesh file does not exist: {absolute_path}"
            )));
        }

        Box::new(Mesh::new(absolute_path))
    } else {
        return Err(ParseError::GeometryWithoutShape(link_name.clone()));
    };

    // extract the origin from the visual node
    let parent_frame = &model.frames[parent_frame_id];
    let placement = parent_frame.placement * parse_origin(node)?;

    // extract the material color
    let mut color = Color::new(0.9, 0.9, 0.9, 1.0);
    if geometry_type != GeometryType::Visual {
        // default color for collision geometries (see above)
    }
    // if there is a material node
    else if let Some(material_node) = node.children().find(|n| n.has_tag_name("material")) {
        // if this material node has a name
        // and this material was already defined in the robot node
        if let Some(material_name) = material_node.attribute("name")
            && let Some(material_color) = materials.get(material_name)
        {
            color = *material_color;
        }
        // else, check if it has a color node
        else if let Some(color_node) = material_node.children().find(|n| n.has_tag_name("color"))
            && let Ok(rgba) = extract_parameter_list::<f64>("rgba", &color_node, Some(4))
        {
            color = Color::new(rgba[0], rgba[1], rgba[2], rgba[3]);

            // if the material has a name, we add it to the materials map
            if let Some(material_name) = material_node.attribute("name")
                && !material_name.is_empty()
            {
                materials.insert(material_name.to_string(), color);
            }
        }
    }

    let geom_obj = GeometryObject::new(
        link_name,
        parent_joint_id,
        parent_frame_id,
        shape,
        color,
        placement,
    );
    Ok(geom_obj)
}

/// Extracts a parameter from the XML node with the given name and converts it to the specified type.
/// Returns an error if the parameter is missing or cannot be parsed.
fn extract_parameter<T: FromStr>(name: &str, node: &roxmltree::Node) -> Result<T, ParseError> {
    node.attribute(name)
        .ok_or_else(|| ParseError::MissingParameter(name.to_string()))?
        .parse::<T>()
        .map_err(|_| ParseError::InvalidParameter(name.to_string()))
}

/// Extracts a list of parameters from the XML node with the given name and converts them to the specified type.
/// Returns an error if the parameter is missing or any value cannot be parsed.
fn extract_parameter_list<T: FromStr>(
    name: &str,
    node: &roxmltree::Node,
    expected_length: Option<usize>,
) -> Result<Vec<T>, ParseError> {
    let vector = node
        .attribute(name)
        .ok_or_else(|| ParseError::MissingParameter(name.to_string()))?
        .split_whitespace()
        .map(|s| {
            s.parse::<T>()
                .map_err(|_| ParseError::InvalidParameter(name.to_string()))
        })
        .collect::<Result<Vec<T>, ParseError>>()?;
    if let Some(expected_length) = expected_length
        && vector.len() != expected_length
    {
        return Err(ParseError::InvalidParameter(name.to_string()));
    }
    Ok(vector)
}

fn parse_origin(node: &roxmltree::Node) -> Result<SE3, ParseError> {
    let isometry = if let Some(origin_node) = node.children().find(|n| n.has_tag_name("origin")) {
        let xyz = extract_parameter_list::<f64>("xyz", &origin_node, Some(3))?;
        let rotation = match extract_parameter_list::<f64>("rpy", &origin_node, Some(3)) {
            Ok(rpy) => SpatialRotation::from_euler_angles(rpy[0], rpy[1], rpy[2]),
            Err(ParseError::MissingParameter(_)) => SpatialRotation::identity(),
            Err(e) => return Err(e),
        };
        let translation = Vector3D::new(xyz[0], xyz[1], xyz[2]);

        SE3::from_parts(translation, rotation)
    } else {
        SE3::identity()
    };
    Ok(isometry)
}
