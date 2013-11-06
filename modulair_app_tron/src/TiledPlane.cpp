#include <modulair_app_tron/TiledPlane.h>
using namespace osg;
using namespace modulair;


TiledPlane::TiledPlane( osg::Vec3 ul, osg::Vec3 ur, 
                        osg::Vec3 ll, osg::Vec3 lr,
                        TexImageList* texImages, int texIndex, 
                        osg::Vec3 center,int xx, int yy)
                        : OSGObjectBase()
{   
    // Set Corners
    ul_ = ul;
    ur_ = ur;
    ll_ = ll;
    lr_ = lr;
    
    xdim = xx;
    ydim = yy;
    // Init Scale
    savedScale = this->getScale();
    // Link Texture Pointer
    textureImages_ = texImages;
    texIndex_ = texIndex;
    // Init Geometry
    initGeometry(texIndex_);
    this->setPosition(center);
    tiled = true;

    // IF PLANE COLLISION DOESN'T WORK, HALUK WAS RIGHT SO UNCOMENT THIS
    // osg::Vec3 tester = ul_-lr_;
    // for(int i=0;i<3;i++)
    // {
    //     if( tester[i]==0 )
    //     {

    //             ul_[i]-=0.5;
    //     }

    // }
    this->box.set(ul_,lr_);
}

void TiledPlane::applyNewTexture(int tindex)
{
    texIndex_ = tindex;
    setTexture(texIndex_);
}

bool TiledPlane::triggerBehavior(QString type)
{
    //std::cerr<<"Triggering: "<< type.toStdString()<<std::endl;
    if(type == "active"){
        setTexture(texIndex_+1);
        return true;
        }else if(type == "idle"){
                setTexture(texIndex_);
                return true;
        }
}

void TiledPlane::initGeometry(int index)
{
        this->addChild(generatePlane(index));
}

osg::Node* TiledPlane::generatePlane(int index)
{
    // osg::Vec3 top_left = ul_;
    // osg::Vec3 bottom_left = ll_;
    // osg::Vec3 bottom_right = lr_;
    // osg::Vec3 top_right = ur_;
    
    // // create geometry
    // osg::Geometry* geom = new osg::Geometry;
    
    // osg::Vec3Array* vertices = new osg::Vec3Array(4);
    // (*vertices)[0] = bottom_left;
    // (*vertices)[1] = bottom_right;
    // (*vertices)[2] = top_right;
    // (*vertices)[3] = top_left;
    
    // geom->setVertexArray(vertices);
    
    // osg::Vec2Array* texcoords = new osg::Vec2Array(4);
    // (*texcoords)[0].set(0.0f, 0.0f);
    // (*texcoords)[1].set(1.0f, 0.0f);
    // (*texcoords)[2].set(1.0f, 1.0f);
    // (*texcoords)[3].set(0.0f, 1.0f);
    // geom->setTexCoordArray(0,texcoords);
    
    // osg::Vec3Array* normals = new osg::Vec3Array(1);
    // (*normals)[0].set(0.0f,-1.0f,0.0f);
    // geom->setNormalArray(normals);
    // geom->setNormalBinding(osg::Geometry::BIND_OVERALL);
    
    // osg::Vec4Array* colors = new osg::Vec4Array(1);
    // (*colors)[0].set(1.0f,1.0f,1.0f,1.0f);
    // geom->setColorArray(colors);
    // geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    
    // geom->addPrimitiveSet(new osg::DrawArrays(GL_QUADS, 0, 4));
    
    // // disable display list so our modified tex coordinates show up
    // geom->setUseDisplayList(false);

    // osg::TexMat* texmat = new osg::TexMat;
    // texmat->setScaleByTextureRectangleSize(true);
    
    // // setup state
    // osg::StateSet* state = geom->getOrCreateStateSet();
    // state->setTextureAttributeAndModes(0, this->textures_->at(index), osg::StateAttribute::ON);
    // state->setTextureAttributeAndModes(0, texmat, osg::StateAttribute::ON);
    
    // // turn off lighting 
    // state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    // // turn on blending
    // state->setMode(GL_BLEND, osg::StateAttribute::ON);

    // state->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

    // // install 'update' callback
    // osg::Geode* geode = new osg::Geode;
    // geode->addDrawable(geom);
    
    // return geode;

    osg::Vec3 top_left = ul_;
    osg::Vec3 bottom_left = ll_;
    osg::Vec3 bottom_right = lr_;
    osg::Vec3 top_right = ur_;
    
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


    // setup state
    osg::StateSet* state = geom->getOrCreateStateSet();

    texture_ = new osg::Texture2D;
    texture_->setDataVariance(Object::DYNAMIC);
    texture_->setFilter(Texture2D::MIN_FILTER, Texture2D::LINEAR_MIPMAP_LINEAR);
    texture_->setFilter(Texture2D::MAG_FILTER, Texture2D::LINEAR);
    texture_->setWrap(Texture::WRAP_S, Texture::REPEAT);
    texture_->setWrap(Texture::WRAP_T, Texture::REPEAT);
    texture_->setImage(textureImages_->at(texIndex_));
    state->setTextureAttributeAndModes(0, texture_, osg::StateAttribute::ON);

    //I use a texmat to repeat the texture ten times.
    osg::TexMat* texmat = new osg::TexMat(osg::Matrix::scale(xdim,ydim,1.0));
    state->setTextureAttributeAndModes(0, texmat);
    
    // turn off lighting 
    state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    // turn on blending
    state->setMode(GL_BLEND, osg::StateAttribute::ON);

    // state->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

    // install 'update' callback
    osg::Geode* geode = new osg::Geode;
    geode->addDrawable(geom);
    
    return geode;
}

void TiledPlane::setTransparency(double val)
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
    
    osg::StateSet* ss = node_ref->getOrCreateStateSet();
    ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
      
}     
}

void TiledPlane::setTexture(int index)
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
    if(this->textures_->at(index) == NULL){
            std::cout<<"Texture is null"<<std::endl;
    }else{
            state->setTextureAttributeAndModes( 0, 
            this->textures_->at(index), 
            osg::StateAttribute::ON);
            state->setMode(GL_BLEND, osg::StateAttribute::ON);
            state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
            state->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);    
    }    
}        
}