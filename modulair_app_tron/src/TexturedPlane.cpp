#include <modulair_app_tron/TexturedPlane.h>
// using namespace modulair;
#include <ros/ros.h>  


namespace modulair{
TexturedPlane::TexturedPlane(   osg::Vec3 ul, osg::Vec3 ur, 
                                osg::Vec3 ll, osg::Vec3 lr,
                                TextureList* tex, int texIndex)
  : OSGObjectBase()
{   
  // Set Corners
  ul_ = ul;
  ur_ = ur;
  ll_ = ll;
  lr_ = lr;

  plane_.set(ul_,ur_,ll_);

  // Init Scale
  savedScale = this->getScale();

  // Link Texture Pointer
  _textures = tex;
  _texIndex = texIndex;
  // Init Geometry
  initGeometry(_texIndex);


  this->box.set(ul_,lr_);


}

TexturedPlane::~TexturedPlane()
{

}

TexturedPlane::TexturedPlane(   osg::Vec3 ul, osg::Vec3 ur, 
                                osg::Vec3 ll, osg::Vec3 lr)
  : OSGObjectBase()
{   
  // Set Corners
  ul_ = ul;
  ur_ = ur;
  ll_ = ll;
  lr_ = lr;
  plane_.set(ul_,ur_,ll_);

  // Init Scale
  savedScale = this->getScale();

  this->box.set(ul_,lr_);
}

void TexturedPlane::createWithTexture(TextureList* tex, int texIndex)
{
  // Link Texture Pointer
  _textures = tex;
  _texIndex = texIndex;
  // Init Geometry
  initGeometry(_texIndex);
}

void TexturedPlane::applyNewTexture(int tindex)
{
  _texIndex = tindex;
  setTexture(_texIndex);
}

bool TexturedPlane::triggerBehavior(QString type)
{
  //std::cerr<<"Triggering: "<< type.toStdString()<<std::endl;
  if(type == "active"){
    setTexture(_texIndex+1);
    return true;
  }else if(type == "idle"){
    setTexture(_texIndex);
    return true;
  }
}

void TexturedPlane::initGeometry(int index)
{
  this->addChild(generatePlane(index));


}


osg::Node* TexturedPlane::generatePlane(int index)
{


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

void TexturedPlane::setTransparency(double val)
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

void TexturedPlane::setTexture(int index)
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
