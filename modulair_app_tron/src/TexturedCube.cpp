// #include "TexturedCube.h"   
// using namespace modulair;
// using namespace osg;    

#include <modulair_app_tron/TexturedCube.h>

namespace modulair{
TexturedCube::TexturedCube( double edgelength, TextureList* tex,  
                            int texIndex, osg::Vec3 center)
	: OSGObjectBase()
{   
    double d = edgelength/2;

    // Init Scale
    savedScale = this->getScale();
    // Link Texture Pointer
    _textures = tex;
    _texIndex = texIndex;
    // Init Geometry
    //top
    this->addChild(plane(_texIndex, osg::Vec3(-d,d,-d),osg::Vec3(-d,d,d),osg::Vec3(d,d,-d),osg::Vec3(d,d,d)));
    //bottom
    this->addChild(plane(_texIndex, osg::Vec3(-d,-d,-d),osg::Vec3(-d,-d,d),osg::Vec3(d,-d,-d),osg::Vec3(d,-d,d)));
    //front
    this->addChild(plane(_texIndex,osg::Vec3(d,d,-d),osg::Vec3(d,d,d),osg::Vec3(d,-d,-d),osg::Vec3(d,-d,d)));
    //back
    this->addChild(plane(_texIndex, osg::Vec3(-d,d,d),osg::Vec3(-d,d,-d),osg::Vec3(-d,-d,d),osg::Vec3(-d,-d,-d)));
    //left
    this->addChild(plane(_texIndex, osg::Vec3(-d,d,-d),osg::Vec3(d,d,-d),osg::Vec3(-d,-d,-d),osg::Vec3(d,-d,-d)));
    //right
    this->addChild(plane(_texIndex, osg::Vec3(d,d,d),osg::Vec3(-d,d,d),osg::Vec3(d,-d,d),osg::Vec3(-d,-d,d)));

    this->setPos3DAbs(center);

    this->box.set(osg::Vec3(-d,-d,-d),osg::Vec3(d,d,d));
}

TexturedCube::TexturedCube( double edgelength, TextureList* tex, 
                            int texIndex, osg::Vec3 center, QString wall)
    : OSGObjectBase()
{   
    double d = edgelength/2;

    // Init Scale
    savedScale = this->getScale();
    // Link Texture Pointer
    _textures = tex;
    _texIndex = texIndex;
    // Init Geometry
    if(wall == QString("top")){
        this->addChild(plane(_texIndex, osg::Vec3(-d,d,-d),osg::Vec3(-d,d,d),osg::Vec3(d,d,-d),osg::Vec3(d,d,d)));
        this->box.set(osg::Vec3(-d,d,-d),osg::Vec3(d,d,d));
    }else if(wall == QString("bottom")){
        this->addChild(plane(_texIndex, osg::Vec3(-d,-d,-d),osg::Vec3(-d,-d,d),osg::Vec3(d,-d,-d),osg::Vec3(d,-d,d)));
        this->box.set(osg::Vec3(-d,-d,-d),osg::Vec3(d,-d,d));
    }else if(wall == QString("left")){
        this->addChild(plane(_texIndex, osg::Vec3(-d,d,-d),osg::Vec3(d,d,-d),osg::Vec3(-d,-d,-d),osg::Vec3(d,-d,-d)));
        this->box.set(osg::Vec3(-d,-d,-d),osg::Vec3(d,-d,-d));
    }else if(wall == QString("right")){
        this->addChild(plane(_texIndex, osg::Vec3(d,d,d),osg::Vec3(-d,d,d),osg::Vec3(d,-d,d),osg::Vec3(-d,-d,d)));
        this->box.set(osg::Vec3(d,d,d),osg::Vec3(-d,-d,d));
    }else if(wall == QString("front")){
        this->addChild(plane(_texIndex, osg::Vec3(d,d,-d),osg::Vec3(d,d,d),osg::Vec3(d,-d,-d),osg::Vec3(d,-d,d)));
        this->box.set(osg::Vec3(d,d,-d),osg::Vec3(d,-d,d));
    }else if(wall == QString("back")){
        this->addChild(plane(_texIndex, osg::Vec3(-d,d,d),osg::Vec3(-d,d,-d),osg::Vec3(-d,-d,d),osg::Vec3(-d,-d,-d)));
        this->box.set(osg::Vec3(-d,d,d),osg::Vec3(-d,-d,-d));
    } 
    this->setPos3DAbs(center);
}

TexturedCube::TexturedCube( double edgelength, TextureList* tex, int top, int bottom, 
                                int left, int back, int right, int front, osg::Vec3 center)
    : OSGObjectBase()
{
    double d = edgelength/2;

    // Init Scale
    savedScale = this->getScale();
    // Link Texture Pointer
    _textures = tex;
    _texIndex = -1;
    // Init Geometry

    // NOTE Planes face inwards to map to skybox
    //top_left
    this->addChild(plane(top, osg::Vec3(d,d,-d),osg::Vec3(d,d,d),osg::Vec3(-d,d,-d),osg::Vec3(-d,d,d)));
    //bottom
    // this->addChild(plane(bottom, Vec3(-d,-d,-d),Vec3(-d,-d,d),Vec3(d,-d,-d),Vec3(d,-d,d)));
    //front
    this->addChild(plane(front, osg::Vec3(d,d,d),osg::Vec3(d,d,-d),osg::Vec3(d,-d,d),osg::Vec3(d,-d,-d)));
    //back
    this->addChild(plane(back, osg::Vec3(-d,d,-d),osg::Vec3(-d,d,d),osg::Vec3(-d,-d,-d),osg::Vec3(-d,-d,d)));
    //left
    this->addChild(plane(left, osg::Vec3(d,d,-d),osg::Vec3(-d,d,-d),osg::Vec3(d,-d,-d),osg::Vec3(-d,-d,-d)));
    //right
    this->addChild(plane(right, osg::Vec3(-d,d,d),osg::Vec3(d,d,d),osg::Vec3(-d,-d,d),osg::Vec3(d,-d,d)));

    this->setPos3DAbs(center);

    this->box.set(osg::Vec3(-d,-d,-d),osg::Vec3(d,d,d));
}

TexturedCube::TexturedCube( double edgelength, double height, TextureList* tex, int top, int bottom, 
                                int left, int back, int right, int front, osg::Vec3 center)
    : OSGObjectBase()
{
    double d = edgelength/2;
    double h = height;
    // Init Scale
    savedScale = this->getScale();
    // Link Texture Pointer
    _textures = tex;
    _texIndex = -1;
    // Init Geometry

    // NOTE Planes face inwards to map to skybox
    //top
    this->addChild(plane(top, osg:: Vec3(d,h,-d),osg::Vec3(d,h,d),osg::Vec3(-d,h,-d),osg::Vec3(-d,h,d)));
    //bottom
    // this->addChild(plane(bottom, Vec3(-d,-d,-d),Vec3(-d,-d,d),Vec3(d,-d,-d),Vec3(d,-d,d)));
    //frontosg::
    this->addChild(plane(front, osg::Vec3(d,h,d),osg::Vec3(d,h,-d),osg::Vec3(d,-h,d),osg::Vec3(d,-h,-d)));
    //back
    this->addChild(plane(back, osg::Vec3(-d,h,-d),osg::Vec3(-d,h,d),osg::Vec3(-d,-h,-d),osg::Vec3(-d,-h,d)));
    //left
    this->addChild(plane(left, osg::Vec3(d,h,-d),osg::Vec3(-d,h,-d),osg::Vec3(d,-h,-d),osg::Vec3(-d,-h,-d)));
    //right
    this->addChild(plane(right, osg::Vec3(-d,h,d),osg::Vec3(d,h,d),osg::Vec3(-d,-h,d),osg::Vec3(d,-h,d)));

    this->setPos3DAbs(center);

    this->box.set(osg::Vec3(-d,-d,-d),osg::Vec3(d,d,d));
}

void TexturedCube::applyNewTexture(int tindex)
{
    _texIndex = tindex;
    setTexture(_texIndex);
}

osg::Node* TexturedCube::plane( int index, 
                                osg::Vec3 tl, osg::Vec3 tr,
                                osg::Vec3 bl, osg::Vec3 br)
{
    osg::Vec3 top_left = tl;
    osg::Vec3 bottom_left = bl;
    osg::Vec3 bottom_right = br;
    osg::Vec3 top_right = tr;
    
    // create geometry
    osg::Geometry* geom = new osg::Geometry;
    
    osg::Vec3Array* vertices = new osg::Vec3Array(4);
    (*vertices)[0] = bottom_left;
    (*vertices)[1] = bottom_right;
    (*vertices)[2] = top_right;
    (*vertices)[3] = top_left;
    
    geom->setVertexArray(vertices);
    
    osg::Vec2Array* texcoords = new osg::Vec2Array(4);
    (*texcoords)[0].set(0.0f, 0.0f);
    (*texcoords)[1].set(1.0f, 0.0f);
    (*texcoords)[2].set(1.0f, 1.0f);
    (*texcoords)[3].set(0.0f, 1.0f);
    geom->setTexCoordArray(0,texcoords);
    
    osg::Vec3Array* normals = new osg::Vec3Array(1);
    (*normals)[0].set(0.0f,-1.0f,0.0f);
    geom->setNormalArray(normals);
    geom->setNormalBinding(osg::Geometry::BIND_OVERALL);
    
    osg::Vec4Array* colors = new osg::Vec4Array(1);
    (*colors)[0].set(1.0f,1.0f,1.0f,1.0f);
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    
    geom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));
    
    // disable display list so our modified tex coordinates show up
    geom->setUseDisplayList(false);
        
    osg::TexMat* texmat = new osg::TexMat;
    texmat->setScaleByTextureRectangleSize(true);
    
    // setup state
    osg::StateSet* state = geom->getOrCreateStateSet();
    state->setTextureAttributeAndModes(0, this->_textures->at(index), osg::StateAttribute::ON);
    state->setTextureAttributeAndModes(0, texmat, osg::StateAttribute::ON);
    
    // turn off lighting 
    state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    // turn on blending
    state->setMode(GL_BLEND, osg::StateAttribute::ON);

    state->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

    // install 'update' callback
    osg::Geode* geode = new osg::Geode;
    geode->addDrawable(geom);
    
    return geode;
}

void TexturedCube::setTransparency(double val)
{
    osg::Node* node_ref;
    osg::Geode* geode_ref;
    osg::Geometry* geometry_ref;

    node_ref = this->getChild(0);
    if(node_ref->asGroup() != 0){
        std::cout<<"Cannot set texture of a group."<<std::endl;
    }else{
        if(node_ref->asGeode() != 0){
            geode_ref = node_ref->asGeode();
        }else{
            std::cout<<"Not a geode"<<std::endl;
            return;   
        }
        if (geode_ref->getDrawable(0)->asGeometry() != 0){
            geometry_ref = geode_ref->getDrawable(0)->asGeometry();
        }else{
            std::cout<<"Not geometry"<<std::endl;
            return;
        }
        osg::Vec4Array* colors = new osg::Vec4Array(1);
        (*colors)[0].set(1.0f,1.0f,1.0f,val);
        geometry_ref->setColorArray(colors);
        geometry_ref->setColorBinding(osg::Geometry::BIND_OVERALL);   
    }     
}

void TexturedCube::setTexture(int index)
{
    osg::Node* node_ref;
    osg::Geode* geode_ref;
    osg::Geometry* geometry_ref;

    node_ref = this->getChild(0);
    if(node_ref->asGroup() != 0){
        std::cout<<"Cannot set texture of a group."<<std::endl;
    }else{
        if(node_ref->asGeode() != 0){
            geode_ref = node_ref->asGeode();
        }else{
            std::cout<<"Not a geode"<<std::endl;
            return;   
        }
        if (geode_ref->getDrawable(0)->asGeometry() != 0){
            geometry_ref = geode_ref->getDrawable(0)->asGeometry();
        }else{
            std::cout<<"Not geometry"<<std::endl;
            return;
        }
        osg::StateSet* state = geometry_ref->getOrCreateStateSet();
        if(this->_textures->at(index) == NULL){
            std::cout<<"Texture is null"<<std::endl;
        }else{
            state->setTextureAttributeAndModes( 0, 
                                            this->_textures->at(index), 
                                            osg::StateAttribute::ON);
            state->setMode(GL_BLEND, osg::StateAttribute::ON);
         state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
         state->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);    
        }    
    }        
}
}