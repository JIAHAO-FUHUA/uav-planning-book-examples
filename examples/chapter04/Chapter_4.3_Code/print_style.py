"""Final grayscale export for publication; geometry and numerical data are unchanged."""
import numpy as np
from matplotlib import colors
from matplotlib.image import AxesImage
from matplotlib.text import Text

def gray_rgba(value):
    a=np.asarray(colors.to_rgba_array(value),dtype=float)
    if not len(a):return a
    luminance=a[:,:3] @ np.array([.2126,.7152,.0722])
    a[:,:3]=luminance[:,None]
    return a[0] if len(a)==1 else a

def make_print_ready(fig):
    # Explicit line styles and patterns are assigned in each teaching figure.
    for artist in fig.findobj():
        if isinstance(artist,AxesImage):
            pixels=np.asarray(artist.get_array())
            if pixels.ndim==3 and pixels.shape[-1]>=3:
                scale=255. if np.issubdtype(pixels.dtype,np.integer) else 1.
                gray=pixels[:,:,:3].astype(float) @ np.array([.2126,.7152,.0722])/scale
                artist.set_data(gray);artist.set_cmap('gray');artist.set_clim(0,1)
        if isinstance(artist,Text):
            try:
                rgba=colors.to_rgba(artist.get_color())
                artist.set_color('white' if min(rgba[:3])>.99 else 'black')
            except (TypeError,ValueError):pass
            continue
        for getter,setter in [('get_color','set_color'),('get_facecolor','set_facecolor'),
                              ('get_edgecolor','set_edgecolor'),
                              ('get_markerfacecolor','set_markerfacecolor'),
                              ('get_markeredgecolor','set_markeredgecolor')]:
            if hasattr(artist,getter) and hasattr(artist,setter):
                try:getattr(artist,setter)(gray_rgba(getattr(artist,getter)()))
                except (TypeError,ValueError,AttributeError):pass
