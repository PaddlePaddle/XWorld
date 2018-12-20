#version 330 core
out float FragColor;

in vec2 TexCoords;

uniform sampler2D ssaoInput;
uniform sampler2D gPosition;

void main() 
{
    vec2 texelSize = 1.0 / vec2(textureSize(ssaoInput, 0));
    float result = 0.0;

    float center_depth = texture(gPosition, TexCoords).a;
    float center_value = texture(ssaoInput, TexCoords).r;

    for (int x = -2; x < 2; ++x) 
    {
        for (int y = -2; y < 2; ++y) 
        {
            vec2 offset = vec2(float(x), float(y)) * texelSize;
            
            float v = texture(ssaoInput, TexCoords + offset).r;
            float d = texture(gPosition, TexCoords + offset).a;
            
            if(abs(center_depth - d) > 0.0002) {
                result += center_value;
            } else {
                result += v;
            }

        }
    }
    FragColor = result / (4.0 * 4.0);
}  