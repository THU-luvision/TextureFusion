#version 130

in vec4 position;
in vec4 color;
in vec4 normal;

uniform mat4 MVP;
uniform mat4 pose;
uniform float threshold;
uniform int colorType;
uniform int time;
uniform int timeDelta;

uniform float materialShininess;
uniform vec4 materialAmbient;
uniform vec4 materialDiffuse;
uniform vec4 materialSpecular;
uniform vec4 lightAmbient;
uniform vec4 lightDiffuse;
uniform vec4 lightSpecular;

out vec4 vColor;
out vec4 vNormal;

#include "color.glsl"

void main()
{
    if(position.w > threshold)
    {
        if(colorType == 1)
        {
            vColor = vec4(-normal.xyz, 1.0);
        }
        else if(colorType == 2)
        {
            vColor = vec4(decodeColor(color.x), 1.0);
        }
		else if(colorType == 3 || colorType == 4)   // texture visualization
		{
			vColor = color;
			if (normal.w != 0.0f) vColor = vec4(decodeColor(color.x), 1.0);
			vNormal = vec4(decodeSignedColor(color.y),normal.w); 
		}
        else
        {
            // use Phong shading

            vec4 material = materialDiffuse;
	    
            vec3 eyeDir = normalize(position.xyz);
			vec4 light_dir_vec4 = vec4(eyeDir,1.0);
	        vec3 light_dir = light_dir_vec4.xyz; 

            vec3 R = normalize(reflect(-normalize(light_dir), normal.xyz));

            vec4 res = lightAmbient  * materialAmbient                                                       // Ambient
                + lightDiffuse  * material * max(dot(normal.xyz, -normalize(light_dir)), 0.0)                  // Diffuse
                + lightSpecular * materialSpecular * pow(max(dot(R, eyeDir), 0.0f), materialShininess); // Specular

            vColor = clamp(res, 0.0, 1.0);
        }
	    gl_Position = MVP * pose * vec4(position.xyz, 1.0);
    }
    else
    {
        gl_Position = vec4(-10, -10, 0, 1);
    }
}
