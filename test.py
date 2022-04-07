topOfLinkMenu = 90
bottomOfLinkMenu = 225
numberOfLinkSections = 5
widthOfEachLinkSection = (bottomOfLinkMenu-topOfLinkMenu)/numberOfLinkSections
linkSections = []
for i in range(1,numberOfLinkSections):
    linkSections.append(int(topOfLinkMenu+i*widthOfEachLinkSection))
print(linkSections)