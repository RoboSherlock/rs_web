/*
 * Copyright 2013 Google Inc.
 *
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 *
 *   Author: Martin Gorner (mgorner@google.com)
 */

var renderer;
var camera;
var currentScene = 0;
var nbScenes = 1;
var sceneDistance = 300;
var sceneTransitionT0;
var sceneTransitionV0;
var sceneTransitionDuration = 500;

var androidscale = new THREE.Vector3(3, 3, 3);

document.addEventListener("DOMContentLoaded", startTHREE);
window.addEventListener('resize', resizeTHREE);

function startTHREE()
{
    var container = document.getElementById('canvas-container');
    renderer = new THREE.WebGLRenderer ( {antialias: true, alpha: true});
    renderer.setSize(container.clientWidth, container.clientHeight);

    // THREE.js creates the 3D <canvas> element for you
    container.appendChild(renderer.domElement);

    // make it pretty (black and transparent)
    renderer.setClearColor(0x000000, 0);
    renderer.clear();

    // CAMERA: field of view (angle), aspect ratio, near, far
    var aspect = container.clientWidth / container.clientHeight;
    camera = new THREE.PerspectiveCamera(35, aspect, 1, 3000);
    camera.position.copy(cameraPositionsForScene(currentScene));

    var scene = new THREE.Scene();

    // STUFF
    createLights(scene);
    createScene2(scene);

    // ANIMATION LOOP
    function animate()
    {
    	var t = new Date().getTime();
        var scale;
        scene.traverse(function(obj) {
            if (obj instanceof THREE.Object3D)
            {
                if(obj.name == "scene-2-textured-cube")
                {
                        obj.position.set(0,0,0);
                        obj.rotation.set(t/5000, t/5000, t/5000);
                }
            }
        })

        renderer.render(scene, camera);
        // let the browser decide the tempo
        requestAnimationFrame(animate);
    }
    animate();
}


//-----------------------------------------------------------------------------------

function createScene2(scene)
{
    var texture = THREE.ImageUtils.loadTexture('static/images/logos/rs_logo_only_bkg.png');

    var geo = new THREE.BoxGeometry(50, 50, 50); // w, h, d
    var mat = new THREE.MeshLambertMaterial({map: texture});
    var cube = new THREE.Mesh(geo, mat);
    cube.name = "scene-2-textured-cube";
    scene.add(cube);
}

function createLights(scene)
{
    var light1 = new THREE.DirectionalLight(0xffffff, 0.6); // color, intens.
    light1.position.set(-1, -1, 0.3); // SW directional light

    var light2 = new THREE.DirectionalLight(0xffffff, 0.6); // color, intens.
    light2.position.set(200, 200, 300); // NE point light

    var light3 = new THREE.DirectionalLight(0xffffff, 0.6); // color, intens.
    light3.position.set(0, 0, 1); // frontal light

    scene.add(light1);
    scene.add(light2);
    scene.add(light3); // add them all
}

//-----------------------------------------------------------------------------------

function resizeTHREE()
{
    var container = document.getElementById('canvas-container');
    if (container !== undefined && renderer !== undefined && camera !== undefined)
    {
        renderer.setSize(container.clientWidth, container.clientHeight);
        camera.aspect	= container.clientWidth/ container.clientHeight;
        camera.updateProjectionMatrix();
        renderer.clear();
    }
}

//-----------------------------------------------------------------------------------

function cameraPositionsForScene(sceneNumber)
{
    var v = new THREE.Vector3();
    v.x = sceneNumber * sceneDistance;
    v.z = 300;
    return v;
}
