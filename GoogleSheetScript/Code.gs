function doGet() {
  var sheet = SpreadsheetApp.openById("1428LzF31YaOr3RqVh0RwDgnonkXH7PljWRaetWMqycw").getActiveSheet();
  var data = sheet.getDataRange().getValues(); 
  var jsonData = JSON.stringify(data);
  Logger.log(jsonData);
  return ContentService.createTextOutput(jsonData).setMimeType(ContentService.MimeType.JSON);
}
        
