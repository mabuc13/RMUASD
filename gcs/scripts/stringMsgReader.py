
class csvText(object):
    def __init__(self, text):
        self.text = text
        splitText = text.split(',')
        self.splitText = []
        for t in splitText:
            self.splitText.append(t.split('='))
    def valueOf(self,value):
    	ret = 'NULL'
    	for t in self.splitText:
    			if value in t[0]:
    			  ret = t[1]
    	return ret
    def get_text():
        return self.text    
