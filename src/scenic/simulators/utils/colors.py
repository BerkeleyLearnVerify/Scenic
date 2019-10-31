
"""Color type (used for car colors in GTA and Webots)."""

import colorsys
from collections import namedtuple

from scenic.core.distributions import Distribution, Range, Normal, Options, toDistribution
from scenic.core.lazy_eval import valueInContext

class Color(namedtuple('Color', ['r', 'g', 'b'])):
	@classmethod
	def withBytes(cls, color):
		return cls._make(c / 255.0 for c in color)

	@staticmethod
	def realToByte(color):
		return tuple(int(round(255 * c)) for c in color)

	@staticmethod
	def uniformColor():
		return toDistribution(Color(Range(0, 1), Range(0, 1), Range(0, 1)))

	@staticmethod
	def defaultCarColor():
		"""Base color distribution estimated from 2012 DuPont survey archived at:
		https://web.archive.org/web/20121229065631/http://www2.dupont.com/Media_Center/en_US/color_popularity/Images_2012/DuPont2012ColorPopularity.pdf"""
		baseColors = {
			(248, 248, 248): 0.24,	# white
			(50, 50, 50): 0.19,		# black
			(188, 185, 183): 0.16,	# silver
			(130, 130, 130): 0.15,	# gray
			(194, 92, 85): 0.10,	# red
			(75, 119, 157): 0.07,	# blue
			(197, 166, 134): 0.05,	# brown/beige
			(219, 191, 105): 0.02,	# yellow/gold
			(68, 160, 135): 0.02,	# green
		}
		converted = { Color.withBytes(color): prob for color, prob in baseColors.items() }
		baseColor = Options(converted)
		# TODO improve this?
		hueNoise = Normal(0, 0.1)
		satNoise = Normal(0, 0.1)
		lightNoise = Normal(0, 0.1)
		return NoisyColorDistribution(baseColor, hueNoise, satNoise, lightNoise)

class NoisyColorDistribution(Distribution):
	def __init__(self, baseColor, hueNoise, satNoise, lightNoise):
		super().__init__(baseColor, hueNoise, satNoise, lightNoise, valueType=Color)
		self.baseColor = baseColor
		self.hueNoise = hueNoise
		self.satNoise = satNoise
		self.lightNoise = lightNoise

	@staticmethod
	def addNoiseTo(color, hueNoise, lightNoise, satNoise):
		hue, lightness, saturation = colorsys.rgb_to_hls(*color)
		hue = max(0, min(1, hue + hueNoise))
		lightness = max(0, min(1, lightness + lightNoise))
		saturation = max(0, min(1, saturation + satNoise))
		return colorsys.hls_to_rgb(hue, lightness, saturation)

	def sampleGiven(self, value):
		bc = value[self.baseColor]
		return Color(*self.addNoiseTo(bc, value[self.hueNoise],
		    value[self.lightNoise], value[self.satNoise]))

	def evaluateInner(self, context):
		self.baseColor = valueInContext(self.baseColor, context)
		self.hueNoise = valueInContext(self.hueNoise, context)
		self.satNoise = valueInContext(self.satNoise, context)
		self.lightNoise = valueInContext(self.lightNoise, context)
