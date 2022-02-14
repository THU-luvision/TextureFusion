
#version 130 

uniform sampler2D ftex;
uniform int colorType;

in vec4 vColor;
in vec4 vNormal;
out vec4 FragColor;

#include "color.glsl"

void main()
{
  if(colorType == 3)
  {
     	vec2 ftexcoord = vec2(vColor.zw);
		 FragColor = vec4(texture2D(ftex, ftexcoord).xyz + vNormal.xyz, 1.0f);
		 if (vNormal.w != 0.0f) FragColor = vColor;
  }
  else if(colorType == 4)
  {
     	vec2 ftexcoord = vec2(vColor.zw);
		 FragColor = texture2D(ftex, ftexcoord);
     if (vNormal.w != 0.0f) FragColor = vColor;
  }
  else
  {
     FragColor = vColor;
  }
}
