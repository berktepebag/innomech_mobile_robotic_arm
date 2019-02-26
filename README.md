# INNOMECH Master Degree - Mobile Robotic Arm for Asbestos Removal

## What is Asbestos and why it is harmful?
Asbestos is a naturally occurring mineral substance that can be pulled into a fluffy consistency. Asbestos fibers are soft and flexible yet resistant to heat, electricity and chemical corrosion. Pure asbestos is an effective insulator, and it can also be mixed into cloth, paper, cement, plastic and other materials to make them stronger. 

These qualities once made asbestos very profitable for business, but unfortunately, they also make asbestos highly toxic.

(https://www.asbestos.com/asbestos/)

## INNOMECH - Mobile Robotic Arm

To remove asbestos many precautions has to be taken before and after. 
<img width="200" alt="Asbestos Removal - https://cdn.nabholz.com/wp-content/uploads/2017/02/Asbestos-Abatement-Header.jpg" src="../imgs/Asbestos-Abatement-Header.jpg">


## my_imfilter(image, filter):

Instead of using built in filters I built a new filtering function.

Checking if filter is valid, if not throw error.

```
if(fx >= 1 && fy >= 1 && mod(fx,2)==1 && mod(fy,2)==1)
    fprintf('Filter size is valid \n');    
else
    msg = 'Filter size is not valid. Please enter positive odd*odd filter \n';
    error(msg)
    return
end  
```

Calculating padding size according to the filter height and width. Create padded image with '0' as padding.
```
padding_horizontal = (fx-1)/2;
padding_vertical = (fy-1)/2;

padded_img = padarray(image,[padding_horizontal,padding_vertical],0);
```

Creating empty convoluted image to later fill in. For every color depth (Gray=1, Colored=3) filter recalculates pixel values and returns sum as convoluted image pixel.
```
convoluted_img = zeros(size(image));

for d = 1:color_depth
    for i = 1:img_size_x
        for j = 1:img_size_y

          i,i+(2*padding_horizontal),j,j+(2*padding_vertical),d);
           temp = padded_img(i:i+(2*padding_horizontal),j:j+(2*padding_vertical),d) .* filter;
           convoluted_img(i,j,d) = sum(temp(:));
        end
    end
end
```

## gen_hybrid_image( image1, image2, cutoff_frequency ):

Hybrid image generator takes in 2 images and cutoff_frequency. 'cutoff_frequency' is the standard deviation, in pixels, of the Gaussian blur that will remove high frequencies. Tried 3 different values, 3,5,7.

### cutoff_frequency: 3-5-7

#### High Frequency Images

<img width="200" alt="High Frequency Image cutoff_frequency 3" src="/imgs/high_frequencies_3.jpg"><img width="200" alt="High Frequency Image cutoff_frequency 5" src="/imgs/high_frequencies_5.jpg">
<img width="200" alt="High Frequency Image cutoff_frequency 7" src="/imgs/high_frequencies_7.jpg">

As the cutoff_frequency increases from 3 to 7, cat image becomes sharper. To achieve this solution, subtracted low frequency image from the original image.

#### Low Frequency Images

<img width="200" alt="low Frequency Image cutoff_frequency 3" src="/imgs/low_frequencies_3.jpg"><img width="200" alt="low Frequency Image cutoff_frequency 5" src="/imgs/low_frequencies_5.jpg">
<img width="200" alt="low Frequency Image cutoff_frequency 7" src="/imgs/low_frequencies_7.jpg">

As the cutoff_frequency increases from 3 to 7, dog image becomes more blur.

#### Hybrid Images

<img width="200" alt="Hybrid Image cutoff_frequency 3" src="/imgs/hybrid_image_3.jpg"><img width="200" alt="Hybrid Image cutoff_frequency 5" src="/imgs/hybrid_image_5.jpg">
<img width="200" alt="Hybrid Image cutoff_frequency 7" src="/imgs/hybrid_image_7.jpg">

As cutoff_frequency increases low frequency images becomes less visible while high frequency images becomes more visible in the hybrid image.


#### Hybrid Image Scales

<img width="600" alt="Hybrid Image cutoff_frequency 3" src="/imgs/hybrid_image_scales_3.jpg">
<img width="600" alt="Hybrid Image cutoff_frequency 5" src="/imgs/hybrid_image_scales_5.jpg">
<img width="600" alt="Hybrid Image cutoff_frequency 7" src="/imgs/hybrid_image_scales_7.jpg">

From scaling hybrid images it can be seen that when cutoff_frequency is 5, selected cat and dog images blends well. 